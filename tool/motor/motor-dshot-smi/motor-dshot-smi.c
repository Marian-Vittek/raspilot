//
// This project generates Dshot protocol on Raspberry Pi pinout.  It
// is based on the work of Jeremy P Bentham on how to use SMI and DMA
// on Raspberry Pi.  I took permission to deliberately copy and modify
// Jeremy's code.
//
// This implementation generates good Dshot signal even if CPU is
// highly loaded by other tasks. On the other hand, it only works on
// 18 GPIOs between GPIO 8 and GPIO 25 and it has more complicated and
// hardware dependent implementation
//
// Marian Vittek.
//
// Here follows the original copyright and code fragments
// ---------------------------------------------------------------------
//
// For detailed description, see https://iosoft.blog
//
// Copyright (c) 2020 Jeremy P Bentham
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// v0.01 JPB 16/7/20  Derived from rpi_smi_test v0.20
//                    Tidied up for github
//




//////////////////////////////////////////////////////////////////////
// Select the dshot version you want to use. Value may be 150, 300,
// 600 or 1200. 


#ifndef DSHOT_VERSION
#define DSHOT_VERSION 300
#endif


//////////////////////////////////////////////////////////////////////
// DMA channel

#define DMA_CHANNEL 14

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

// For each protocol there are 4 values to set up. The clock tick in
// nanoseconds, and 3 timings in ticks (sum of which gives 1/3 of the
// length of T0H dshot period). For example timing 8,10,64,20 means
// that the internal clock tick will last for 8ns and the length of
// T0H bit will be 8*3*(20+64+20) == 2496ns.  Tick length can be an
// even number between 2 and 30, timings in tick have ranges 0-63,
// 0-127, 0-63.

// In real setting I use a bit shorter values for pulses than defined
// in dshot standard. My ESC works better with those values.

#if DSHOT_VERSION == 150

// DSHOT_150, tick == 8ns, T0H == 3 * 104 ticks, in my ESC T0H == 84 ticks
//#define DSHOT_SMI_TIMING 8, 20,64,20
#define DSHOT_SMI_TIMING 8,10,64,10

#elif DSHOT_VERSION == 300

// DSHOT_300, tick == 4ns, T0H == 3 * 104 ticks, in my ESC T0H == 84 ticks
//#define DSHOT_SMI_TIMING 4, 20,64,20
#define DSHOT_SMI_TIMING 4,10,64,10

#elif DSHOT_VERSION == 600

// DSHOT_600, tick 4ns, T0H == 3 * 52 ticks, does not work with my ESC
#define DSHOT_SMI_TIMING 4,10,32,10

#elif DSHOT_VERSION == 1200

// DSHOT_1200, tick 4ns, T0H == 3 * 26 ticks, my ESC can't handle
#define DSHOT_SMI_TIMING  4, 5,16,5

#endif


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "rpi_dma_utils.h"

#include <assert.h>

enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
};

#define DSHOT_NUM_PINS 		18
#define DSHOT_BROADCAST_BYTES 	(8*16*4)
#define TIMESPEC_TO_INT(tt)     (tt.tv_sec * 1000000000LL + tt.tv_nsec)



#define USE_SMI         1
#define USE_DMA         1
#define DISP_ZEROS      1

#define GPIO_PULL 	57 /* Pull up/pull down */
#define GPIO_PULLCLK0 	38 /* Pull up/pull down clock */

#define SMI_A0_PIN      5
#define SMI_SOE_PIN     6
#define SMI_SWE_PIN     7

#define DAC_D0_PIN      8
#define DAC_NPINS       18

//#define NSAMPLES        512

#define SMI_BASE    (rpiRegBase + 0x600000)
#define SMI_CS      0x00    // Control & status
#define SMI_L       0x04    // Transfer length
#define SMI_A       0x08    // Address
#define SMI_D       0x0c    // Data
#define SMI_DSR0    0x10    // Read settings device 0
#define SMI_DSW0    0x14    // Write settings device 0
#define SMI_DSR1    0x18    // Read settings device 1
#define SMI_DSW1    0x1c    // Write settings device 1
#define SMI_DSR2    0x20    // Read settings device 2
#define SMI_DSW2    0x24    // Write settings device 2
#define SMI_DSR3    0x28    // Read settings device 3
#define SMI_DSW3    0x2c    // Write settings device 3
#define SMI_DMC     0x30    // DMA control
#define SMI_DCS     0x34    // Direct control/status
#define SMI_DCA     0x38    // Direct address
#define SMI_DCD     0x3c    // Direct data
#define SMI_FD      0x40    // FIFO debug
#define SMI_REGLEN  (SMI_FD * 4)

#define SMI_DEV     0
#define SMI_8_BITS  0
#define SMI_16_BITS 1
#define SMI_18_BITS 2
#define SMI_9_BITS  3

#define DMA_SMI_DREQ 4

char *smi_regstrs[] = {
    "CS","LEN","A","D","DSR0","DSW0","DSR1","DSW1",
    "DSR2","DSW2","DSR3","DSW3","DMC","DCS","DCA","DCD",""
};

#define STRS(x)     STRS_(x) ","
#define STRS_(...)  #__VA_ARGS__
#define REG_DEF(name, fields) typedef union {struct {volatile uint32_t fields;}; volatile uint32_t value;} name

#define SMI_CS_FIELDS \
    enable:1, done:1, active:1, start:1, clear:1, write:1, _x1:2,\
    teen:1, intd:1, intt:1, intr:1, pvmode:1, seterr:1, pxldat:1, edreq:1,\
    _x2:8, _x3:1, aferr:1, txw:1, rxr:1, txd:1, rxd:1, txe:1, rxf:1  
REG_DEF(SMI_CS_REG, SMI_CS_FIELDS);
#define SMI_L_FIELDS \
    len:32
REG_DEF(SMI_L_REG, SMI_L_FIELDS);
#define SMI_A_FIELDS \
    addr:6, _x1:2, dev:2
REG_DEF(SMI_A_REG, SMI_A_FIELDS);
#define SMI_D_FIELDS \
    data:32
REG_DEF(SMI_D_REG, SMI_D_FIELDS);
#define SMI_DMC_FIELDS \
    reqw:6, reqr:6, panicw:6, panicr:6, dmap:1, _x1:3, dmaen:1
REG_DEF(SMI_DMC_REG, SMI_DMC_FIELDS);
#define SMI_DSR_FIELDS \
    rstrobe:7, rdreq:1, rpace:7, rpaceall:1, rhold:6, fsetup:1, mode68:1, rsetup:6, rwidth:2
REG_DEF(SMI_DSR_REG, SMI_DSR_FIELDS);
#define SMI_DSW_FIELDS \
    wstrobe:7, wdreq:1, wpace:7, wpaceall:1, whold:6, wswap:1, wformat:1, wsetup:6, wwidth:2
REG_DEF(SMI_DSW_REG, SMI_DSW_FIELDS);
#define SMI_DCS_FIELDS \
    enable:1, start:1, done:1, write:1
REG_DEF(SMI_DCS_REG, SMI_DCS_FIELDS);
#define SMI_DCA_FIELDS \
    addr:6, _x1:2, dev:2
REG_DEF(SMI_DCA_REG, SMI_DCA_FIELDS);
#define SMI_DCD_FIELDS \
    data:32
REG_DEF(SMI_DCD_REG, SMI_DCD_FIELDS);
#define SMI_FLVL_FIELDS \
    fcnt:6, _x1:2, flvl:6
REG_DEF(SMI_FLVL_REG, SMI_FLVL_FIELDS);

#define CLK_SMI_CTL     0xb0
#define CLK_SMI_DIV     0xb4

char *smi_cs_regstrs = STRS(SMI_CS_FIELDS);

extern MEM_MAP gpio_regs, dma_regs;
MEM_MAP vc_mem, smi_regs;
extern MEM_MAP clk_regs;

static int sample_count = DSHOT_BROADCAST_BYTES;
static int previousTransfer = 0;

volatile SMI_CS_REG  *smi_cs;
volatile SMI_L_REG   *smi_l;
volatile SMI_A_REG   *smi_a;
volatile SMI_D_REG   *smi_d;
volatile SMI_DMC_REG *smi_dmc;
volatile SMI_DSR_REG *smi_dsr;
volatile SMI_DSW_REG *smi_dsw;
volatile SMI_DCS_REG *smi_dcs;
volatile SMI_DCA_REG *smi_dca;
volatile SMI_DCD_REG *smi_dcd;

#define TX_SAMPLE_SIZE  1       // Number of raw bytes per sample
#define VC_MEM_SIZE(ns) 	(PAGE_SIZE + ((ns)+4)*TX_SAMPLE_SIZE)

// 3D mode, if dshot3dMode != 0 then reverse rotation is enabled
static int dshot3dMode = 0;


//uint8_t sample_buff[NSAMPLES];

void map_devices(void);
void fail(char *s);
void unmap_devices(int sig);
void init_smi(int width, int ns, int setup, int hold, int strobe);
void dma_wait(int chan);


// Map GPIO, DMA and SMI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void)
{
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    map_periph(&clk_regs, (void *)CLK_BASE, PAGE_SIZE);
    map_periph(&smi_regs, (void *)SMI_BASE, PAGE_SIZE);
    memset(smi_regs.virt, 0, SMI_REGLEN);
}

// Catastrophic failure in initial setup
void fail(char *s)
{
    printf(s);
    unmap_devices(0);
}

// Free memory segments and exit
void unmap_devices(int sig)
{
    printf("debug: Motor-dshot-smi Closing\n");
    unmap_periph_mem(&smi_regs);
    unmap_periph_mem(&clk_regs); // it was not there, but probably shall unmap too?    
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
}

// Initialise SMI, given data width, time step, and setup/hold/strobe counts
// Step value is in nanoseconds: even numbers, 2 to 30
// ns is the clock in nanoseconds, setup, strobe, hold in those clock ticks?
// max values seems to be 3, 30, 63, 127, 63
//
void init_smi(int width, int ns, int setup, int strobe, int hold)
{
    int divi = ns / 2;

    smi_cs  = (SMI_CS_REG *) REG32(smi_regs, SMI_CS);
    smi_l   = (SMI_L_REG *)  REG32(smi_regs, SMI_L);
    smi_a   = (SMI_A_REG *)  REG32(smi_regs, SMI_A);
    smi_d   = (SMI_D_REG *)  REG32(smi_regs, SMI_D);
    smi_dmc = (SMI_DMC_REG *)REG32(smi_regs, SMI_DMC);
    smi_dsr = (SMI_DSR_REG *)REG32(smi_regs, SMI_DSR0);
    smi_dsw = (SMI_DSW_REG *)REG32(smi_regs, SMI_DSW0);
    smi_dcs = (SMI_DCS_REG *)REG32(smi_regs, SMI_DCS);
    smi_dca = (SMI_DCA_REG *)REG32(smi_regs, SMI_DCA);
    smi_dcd = (SMI_DCD_REG *)REG32(smi_regs, SMI_DCD);
    smi_cs->value = smi_l->value = smi_a->value = 0;
    smi_dsr->value = smi_dsw->value = smi_dcs->value = smi_dca->value = 0;
    if (*REG32(clk_regs, CLK_SMI_DIV) != divi << 12)
    {
        *REG32(clk_regs, CLK_SMI_CTL) = CLK_PASSWD | (1 << 5);
        usleep(10);
        while (*REG32(clk_regs, CLK_SMI_CTL) & (1 << 7)) ;
        usleep(10);
        *REG32(clk_regs, CLK_SMI_DIV) = CLK_PASSWD | (divi << 12);
        usleep(10);
        *REG32(clk_regs, CLK_SMI_CTL) = CLK_PASSWD | 6 | (1 << 4);
        usleep(10);
        while ((*REG32(clk_regs, CLK_SMI_CTL) & (1 << 7)) == 0) ;
        usleep(100);
    }

    if (smi_cs->seterr)
        smi_cs->seterr = 1;

    smi_dsr->rsetup = smi_dsw->wsetup = setup;
    smi_dsr->rstrobe = smi_dsw->wstrobe = strobe;
    smi_dsr->rhold = smi_dsw->whold = hold;
    smi_dmc->panicr = smi_dmc->panicw = 8;
    smi_dmc->reqr = smi_dmc->reqw = 2;
    smi_dsr->rwidth = smi_dsw->wwidth = width;
}

// Wait until DMA is complete
void dma_wait(int chan) {
    // wait 300us DSHOT frame shall not take more
    if (dma_transfer_len(chan)!=0) usleep(100);
    if (dma_transfer_len(chan)!=0) usleep(100);
    if (dma_transfer_len(chan)!=0) usleep(100);
    if (dma_transfer_len(chan)!=0) printf("debug: DMA transfer timeout\n");
    // printf("DMA transfer sleeps: %d\n", n);
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////


static void dshotDmaSmiSend() {
    DMA_CB 	*cbs;
    uint8_t 	*txdata;

    // first check if the previous transfer finished
    if (previousTransfer && dma_transfer_len(DMA_CHANNEL)!=0) {
	printf("debug: Error: Previous DMA transfer timeout.\n");
    }
    // stop the previous dma
    // [M.V.] I've commented this out. Not sure what it is doing but it generated strange peak on the pin
    //if (smi_regs.virt) *REG32(smi_regs, SMI_CS) = 0;
    stop_dma(DMA_CHANNEL);

    // Start the new transfer
    smi_dsr->rwidth = SMI_8_BITS; 
    smi_l->len = sample_count;
    smi_dmc->dmaen = 1;
    smi_cs->write = 1;
    smi_cs->enable = 1;
    smi_cs->clear = 1;
    // inlined dac_ladder_dma(&vc_mem, framebits, sample_count, 0)
    cbs = vc_mem.virt;
    txdata = (uint8_t *)(cbs+1);
    // memcpy(txdata, framebits, sample_count);
    enable_dma(DMA_CHANNEL);
    cbs[0].ti = DMA_DEST_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_SRCE_INC;
    cbs[0].tfr_len = DSHOT_BROADCAST_BYTES;
    cbs[0].srce_ad = MEM_BUS_ADDR((&vc_mem), txdata);
    cbs[0].dest_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[0].next_cb = 0;
    start_dma(&vc_mem, DMA_CHANNEL, &cbs[0], 0);
    smi_cs->start = 1;
    
    previousTransfer = 1;
    //dma_wait(DMA_CHANNEL);
    //if (smi_regs.virt) *REG32(smi_regs, SMI_CS) = 0;
    //stop_dma(DMA_CHANNEL);
}

void dshotSendFrames(int motorPins[], int motorMax, unsigned frame[]) {
    int         i, j, bi;
    unsigned    bit;
    uint32_t	msk;
    DMA_CB 	*cbs;
    uint32_t 	*txdata;

    cbs = vc_mem.virt;
    txdata = (uint32_t *)(cbs+1);

    // compute masks for zero bits in all frames
    j = 0;
    for(bi=0; bi<16; bi++) {
        bit = (0x8000 >> bi);
	msk = 0;
        for(i=0; i<motorMax; i++) {
            if ((frame[i] & bit) != 0) msk |= (1<<(motorPins[i]-DAC_D0_PIN));
        }
	// 1 dshot bit is 'encoded' into 8 values 3*d0L + 3*data + 2*padding
	j = bi * 8 + 3;
	txdata[j++] = msk;
	txdata[j++] = msk;
	txdata[j++] = msk;
    }
    dshotDmaSmiSend();
}

static int dshotAddChecksumAndTelemetry(int packet, int telem) {
    int packet_telemetry = (packet << 1) | (telem & 1);
    int i;
    int csum = 0;
    int csum_data = packet_telemetry;
    for (i = 0; i < 3; i++) {
        csum ^=  csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    return ((packet_telemetry << 4) | csum);
}

static uint32_t getRpiRegBase(void) {
    const char *revision_file = "/proc/device-tree/system/linux,revision";
    uint8_t revision[4] = { 0 };
    uint32_t cpu = 4;
    FILE *fd;

    if ((fd = fopen(revision_file, "rb")) == NULL) {
        printf("debug Error: Can't open '%s'\n", revision_file);
    } else {
        if (fread(revision, 1, sizeof(revision), fd) == 4) {
            cpu = (revision[2] >> 4) & 0xf;
        } else {
            printf("debug Error: Revision data too short\n");
        }
        fclose(fd);
    }

    // printf("debug CPU type: %d\n", cpu);
    switch (cpu) {
        case 0: // BCM2835 [Pi 1 A; Pi 1 B; Pi 1 B+; Pi Zero; Pi Zero W]
            return(0x20000000);
        case 1: // BCM2836 [Pi 2 B]
        case 2: // BCM2837 [Pi 3 B; Pi 3 B+; Pi 3 A+]
            return(0x3f000000);
        case 3: // BCM2711 [Pi 4 B]
            return(0xfe000000);
        default:
            printf("debug Error: Unrecognised revision code\n");
            return(0xfe000000);
    }
}

static inline uint64_t dshotGetNanoseconds() {
    struct timespec tt;
    clock_gettime(CLOCK_MONOTONIC_RAW, &tt);
    return(TIMESPEC_TO_INT(tt));
}

// Send a command repeatedly during a given perion of time
static void dshotRepeatSendCommand(int motorPins[], int motorMax, int cmd, int telemetry, int64_t timePeriodMsec) {
    unsigned    frame[DSHOT_NUM_PINS+1];
    int         i;
    int64_t     t;
    
    for(i=0; i<motorMax; i++) frame[i] = dshotAddChecksumAndTelemetry(cmd, telemetry);
    t = dshotGetNanoseconds() + timePeriodMsec * 1000000LL;
    while (dshotGetNanoseconds() <= t) {
        dshotSendFrames(motorPins, motorMax, frame);
        usleep(1000);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Main exported functions of the module implementing raspilot motor instance.
//////////////////////////////////////////////////////////////////////////////////////////////

// This function allows to set bidirectional rotation (mode3dFlag!=0) and reverse rotation logic (reverseDirectionFlag!=0).
// Changing 3D mode is interfering with rotation direction (at least on my ESC), so always reset the direction when changing 3D.
// TODO: Allow changing spin direction per motor !
void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) {
    int         repeatMsec;

    repeatMsec = 25;
    
    // This seems to be a delicate operation, I don't know why but it does not work if I do not send
    // stop motors for some time first.

    dshot3dMode = mode3dFlag;
    if (dshot3dMode) {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_3D_MODE_ON, 1, repeatMsec);
    } else {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_3D_MODE_OFF, 1, repeatMsec);
    }
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SAVE_SETTINGS, 0, repeatMsec);

    if (reverseDirectionFlag) {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SPIN_DIRECTION_REVERSED, 1, repeatMsec);
    } else {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SPIN_DIRECTION_NORMAL, 1, repeatMsec);
    }
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SAVE_SETTINGS, 0, repeatMsec);
}

void motorImplementationInitialize(int motorPins[], int motorMax) {
    int 	i,j,k;
    DMA_CB 	*cbs;
    uint32_t 	*txdata;
    void	*mmm;
    
    for(i=0; i<motorMax; i++) {
	if (motorPins[i] < DAC_D0_PIN || motorPins[i] >= DAC_D0_PIN+DAC_NPINS) {
	    printf("debug: wrong motor pin %d. Gpio pins must be in range %d - %d for smi.\n", motorPins[i], DAC_D0_PIN, DAC_D0_PIN+DAC_NPINS-1);
	    fflush(stdout);
	    exit(-1);
	}
    }
    
    rpiRegBase = getRpiRegBase();
    map_devices();
    
    // Initialize SMI to Dshot timing
    init_smi(2, DSHOT_SMI_TIMING);

    // [M.V.] Hmm. Why this was here?
    // gpio_mode(SMI_SOE_PIN, GPIO_ALT1);
    // gpio_mode(SMI_SWE_PIN, GPIO_ALT1);
    smi_cs->clear = 1;
    smi_cs->clear = smi_cs->seterr = smi_cs->aferr=1;
    // smi_cs->enable = 1;
    smi_dcs->enable = 1;
    mmm = map_uncached_mem(&vc_mem, VC_MEM_SIZE(sample_count));
    if (mmm == NULL) {
	printf("debug: can't get uncached memory.\n");
    }
    // Precompute 'zero' dhsot frame
    cbs = vc_mem.virt;
    txdata = (uint32_t *)(cbs+1);
    j = 0;
    for(i=0; i<16; i++) {
	for(k=0; k<3;k++) txdata[j++] = 0xffffffff;
	for(k=0; k<5;k++) txdata[j++] = 0;
    }

    for (i=0; i<motorMax; i++) gpio_mode(motorPins[i], GPIO_ALT1);

    // My esc does not spin anything before it stops initial beeping, so
    // prefer to spend that time here
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_MOTOR_STOP, 0, 2000);    
}

void motorImplementationFinalize(int motorPins[], int motorMax) {
    int i;
    
    printf("debug: motorImplementationFinalize\n");
    stop_dma(DMA_CHANNEL);
    for (i=0; i<motorMax; i++)gpio_mode(motorPins[i], GPIO_IN);
    unmap_periph_mem(&vc_mem);
    unmap_devices(0);
   
}

void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) {
    int         i;
    unsigned    frame[DSHOT_NUM_PINS+1];
    int         val;

    assert(motorMax < DSHOT_NUM_PINS);

    for(i=0; i<motorMax; i++) {
        if (dshot3dMode) {
            // translate double throttles ranging <-1, 1> to dshot frames.
            if (motorThrottle[i] >= 0) {
                val = motorThrottle[i] * 999 + 1048;
            } else {
                val = -motorThrottle[i] * 999 + 48;
            }
        } else {
            // translate double throttles ranging <0, 1> to dshot frames.
            val = motorThrottle[i] * 1999 + 48;
        }
	// printf("val == %d\n", val);
        if (val < 48 || val >= 2048) val = DSHOT_CMD_MOTOR_STOP;
        frame[i] = dshotAddChecksumAndTelemetry(val, 0);
    }

    dshotSendFrames(motorPins, motorMax, frame);
}


