/*!
 * @file DFRobot_BMI160.h
 * @brief Define the basic structure of class DFRobot_BMI160 
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author  DFRobot_haoJ(hao.jiang@dfrobot.com)
 * @version V1.0
 * @date 2017-12-01
 * @url https://github.com/DFRobot/DFRobot_BMI160
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "pi2c.h"
//#include<Arduino.h>
//#include<Wire.h>
//#include<SPI.h>

#undef LITTLE_ENDIAN
#define LITTLE_ENDIAN 1

/** Mask definitions */
#define BMI160_ACCEL_BW_MASK                    UINT8_C(0x70)
#define BMI160_ACCEL_ODR_MASK                   UINT8_C(0x0F)
#define BMI160_ACCEL_UNDERSAMPLING_MASK         UINT8_C(0x80)
#define BMI160_ACCEL_RANGE_MASK                 UINT8_C(0x0F)
#define BMI160_GYRO_BW_MASK                     UINT8_C(0x30)
#define BMI160_GYRO_ODR_MASK                    UINT8_C(0x0F)
#define BMI160_GYRO_RANGE_MSK                   UINT8_C(0x07)

/** Mask definitions for INT_EN registers */
#define BMI160_ANY_MOTION_X_INT_EN_MASK         UINT8_C(0x01)
#define BMI160_HIGH_G_X_INT_EN_MASK             UINT8_C(0x01)
#define BMI160_NO_MOTION_X_INT_EN_MASK          UINT8_C(0x01)
#define BMI160_ANY_MOTION_Y_INT_EN_MASK         UINT8_C(0x02)
#define BMI160_HIGH_G_Y_INT_EN_MASK             UINT8_C(0x02)
#define BMI160_NO_MOTION_Y_INT_EN_MASK          UINT8_C(0x02)
#define BMI160_ANY_MOTION_Z_INT_EN_MASK         UINT8_C(0x04)
#define BMI160_HIGH_G_Z_INT_EN_MASK             UINT8_C(0x04)
#define BMI160_NO_MOTION_Z_INT_EN_MASK          UINT8_C(0x04)
#define BMI160_SIG_MOTION_INT_EN_MASK           UINT8_C(0x07)
#define BMI160_ANY_MOTION_ALL_INT_EN_MASK       UINT8_C(0x07)
#define BMI160_STEP_DETECT_INT_EN_MASK          UINT8_C(0x08)
#define BMI160_DOUBLE_TAP_INT_EN_MASK           UINT8_C(0x10)
#define BMI160_SINGLE_TAP_INT_EN_MASK           UINT8_C(0x20)
#define BMI160_FIFO_FULL_INT_EN_MASK            UINT8_C(0x20)
#define BMI160_ORIENT_INT_EN_MASK               UINT8_C(0x40)
#define BMI160_FIFO_WATERMARK_INT_EN_MASK       UINT8_C(0x40)
#define BMI160_LOW_G_INT_EN_MASK                UINT8_C(0x08)
#define BMI160_STEP_DETECT_EN_MASK              UINT8_C(0x08)
#define BMI160_FLAT_INT_EN_MASK                 UINT8_C(0x80)
#define BMI160_DATA_RDY_INT_EN_MASK             UINT8_C(0x10)

/** Mask definitions for INT_OUT_CTRL register */
#define BMI160_INT1_EDGE_CTRL_MASK              UINT8_C(0x01)
#define BMI160_INT1_OUTPUT_MODE_MASK            UINT8_C(0x04)
#define BMI160_INT1_OUTPUT_TYPE_MASK            UINT8_C(0x02)
#define BMI160_INT1_OUTPUT_EN_MASK              UINT8_C(0x08)
#define BMI160_INT2_EDGE_CTRL_MASK              UINT8_C(0x10)
#define BMI160_INT2_OUTPUT_MODE_MASK            UINT8_C(0x40)
#define BMI160_INT2_OUTPUT_TYPE_MASK            UINT8_C(0x20)
#define BMI160_INT2_OUTPUT_EN_MASK              UINT8_C(0x80)

/** Mask definitions for INT_LATCH register */
#define BMI160_INT1_INPUT_EN_MASK               UINT8_C(0x10)
#define BMI160_INT2_INPUT_EN_MASK               UINT8_C(0x20)
#define BMI160_INT_LATCH_MASK                   UINT8_C(0x0F)

/** Mask definitions for INT_MAP register */
#define BMI160_INT1_LOW_G_MASK                  UINT8_C(0x01)
#define BMI160_INT1_HIGH_G_MASK                 UINT8_C(0x02)
#define BMI160_INT1_SLOPE_MASK                  UINT8_C(0x04)
#define BMI160_INT1_NO_MOTION_MASK              UINT8_C(0x08)
#define BMI160_INT1_DOUBLE_TAP_MASK             UINT8_C(0x10)
#define BMI160_INT1_SINGLE_TAP_MASK             UINT8_C(0x20)
#define BMI160_INT1_FIFO_FULL_MASK              UINT8_C(0x20)
#define BMI160_INT1_FIFO_WM_MASK                UINT8_C(0x40)
#define BMI160_INT1_ORIENT_MASK                 UINT8_C(0x40)
#define BMI160_INT1_FLAT_MASK                   UINT8_C(0x80)
#define BMI160_INT1_DATA_READY_MASK             UINT8_C(0x80)
#define BMI160_INT2_LOW_G_MASK                  UINT8_C(0x01)
#define BMI160_INT1_LOW_STEP_DETECT_MASK        UINT8_C(0x01)
#define BMI160_INT2_LOW_STEP_DETECT_MASK        UINT8_C(0x01)
#define BMI160_INT2_HIGH_G_MASK                 UINT8_C(0x02)
#define BMI160_INT2_FIFO_FULL_MASK              UINT8_C(0x02)
#define BMI160_INT2_FIFO_WM_MASK    UINT8_C(0x04)
#define BMI160_INT2_SLOPE_MASK                  UINT8_C(0x04)
#define BMI160_INT2_DATA_READY_MASK             UINT8_C(0x08)
#define BMI160_INT2_NO_MOTION_MASK              UINT8_C(0x08)
#define BMI160_INT2_DOUBLE_TAP_MASK             UINT8_C(0x10)
#define BMI160_INT2_SINGLE_TAP_MASK             UINT8_C(0x20)
#define BMI160_INT2_ORIENT_MASK                 UINT8_C(0x40)
#define BMI160_INT2_FLAT_MASK                   UINT8_C(0x80)

/** Mask definitions for INT_DATA register */
#define BMI160_TAP_SRC_INT_MASK                 UINT8_C(0x08)
#define BMI160_LOW_HIGH_SRC_INT_MASK            UINT8_C(0x80)
#define BMI160_MOTION_SRC_INT_MASK              UINT8_C(0x80)

/** Mask definitions for INT_MOTION register */
#define BMI160_SLOPE_INT_DUR_MASK               UINT8_C(0x03)
#define BMI160_NO_MOTION_INT_DUR_MASK           UINT8_C(0xFC)
#define BMI160_NO_MOTION_SEL_BIT_MASK           UINT8_C(0x01)

/** Mask definitions for INT_TAP register */
#define BMI160_TAP_DUR_MASK                     UINT8_C(0x07)
#define BMI160_TAP_SHOCK_DUR_MASK               UINT8_C(0x40)
#define BMI160_TAP_QUIET_DUR_MASK               UINT8_C(0x80)
#define BMI160_TAP_THRES_MASK                   UINT8_C(0x1F)

/** Mask definitions for INT_FLAT register */
#define BMI160_FLAT_THRES_MASK                  UINT8_C(0x3F)
#define BMI160_FLAT_HOLD_TIME_MASK              UINT8_C(0x30)
#define BMI160_FLAT_HYST_MASK                   UINT8_C(0x07)

/** Mask definitions for INT_LOWHIGH register */
#define BMI160_LOW_G_HYST_MASK                  UINT8_C(0x03)
#define BMI160_LOW_G_LOW_MODE_MASK              UINT8_C(0x04)
#define BMI160_HIGH_G_HYST_MASK                 UINT8_C(0xC0)

/** Mask definitions for INT_SIG_MOTION register */
#define BMI160_SIG_MOTION_SEL_MASK              UINT8_C(0x02)
#define BMI160_SIG_MOTION_SKIP_MASK             UINT8_C(0x0C)
#define BMI160_SIG_MOTION_PROOF_MASK            UINT8_C(0x30)

/** Mask definitions for INT_ORIENT register */
#define BMI160_ORIENT_MODE_MASK                 UINT8_C(0x03)
#define BMI160_ORIENT_BLOCK_MASK                UINT8_C(0x0C)
#define BMI160_ORIENT_HYST_MASK                 UINT8_C(0xF0)
#define BMI160_ORIENT_THETA_MASK                UINT8_C(0x3F)
#define BMI160_ORIENT_UD_ENABLE                 UINT8_C(0x40)
#define BMI160_AXES_EN_MASK                     UINT8_C(0x80)

/** Mask definitions for FIFO_CONFIG register */
#define BMI160_FIFO_GYRO      UINT8_C(0x80)
#define BMI160_FIFO_ACCEL     UINT8_C(0x40)
#define BMI160_FIFO_AUX       UINT8_C(0x20)
#define BMI160_FIFO_TAG_INT1      UINT8_C(0x08)
#define BMI160_FIFO_TAG_INT2      UINT8_C(0x04)
#define BMI160_FIFO_TIME      UINT8_C(0x02)
#define BMI160_FIFO_HEADER      UINT8_C(0x10)
#define BMI160_FIFO_CONFIG_1_MASK               UINT8_C(0xFE)


/** Mask definitions for STEP_CONF register */
#define BMI160_STEP_COUNT_EN_BIT_MASK           UINT8_C(0x08)
#define BMI160_STEP_DETECT_MIN_THRES_MASK       UINT8_C(0x18)
#define BMI160_STEP_DETECT_STEPTIME_MIN_MASK    UINT8_C(0x07)
#define BMI160_STEP_MIN_BUF_MASK                UINT8_C(0x07)

/** Mask definition for FIFO Header Data Tag */
#define BMI160_FIFO_TAG_INTR_MASK               UINT8_C(0xFC)

/** Fifo byte counter mask definitions */
#define BMI160_FIFO_BYTE_COUNTER_MASK           UINT8_C(0x07)

/** Enable/disable bit value */
#define BMI160_ENABLE                           UINT8_C(0x01)
#define BMI160_DISABLE                          UINT8_C(0x00)

/** Latch Duration */
#define BMI160_LATCH_DUR_NONE                   UINT8_C(0x00)
#define BMI160_LATCH_DUR_312_5_MICRO_SEC        UINT8_C(0x01)
#define BMI160_LATCH_DUR_625_MICRO_SEC          UINT8_C(0x02)
#define BMI160_LATCH_DUR_1_25_MILLI_SEC         UINT8_C(0x03)
#define BMI160_LATCH_DUR_2_5_MILLI_SEC          UINT8_C(0x04)
#define BMI160_LATCH_DUR_5_MILLI_SEC            UINT8_C(0x05)
#define BMI160_LATCH_DUR_10_MILLI_SEC           UINT8_C(0x06)
#define BMI160_LATCH_DUR_20_MILLI_SEC           UINT8_C(0x07)
#define BMI160_LATCH_DUR_40_MILLI_SEC           UINT8_C(0x08)
#define BMI160_LATCH_DUR_80_MILLI_SEC           UINT8_C(0x09)
#define BMI160_LATCH_DUR_160_MILLI_SEC          UINT8_C(0x0A)
#define BMI160_LATCH_DUR_320_MILLI_SEC          UINT8_C(0x0B)
#define BMI160_LATCH_DUR_640_MILLI_SEC          UINT8_C(0x0C)
#define BMI160_LATCH_DUR_1_28_SEC               UINT8_C(0x0D)
#define BMI160_LATCH_DUR_2_56_SEC               UINT8_C(0x0E)
#define BMI160_LATCHED                          UINT8_C(0x0F)

/** BMI160 Register map */
#define BMI160_CHIP_ID_ADDR   UINT8_C(0x00)
#define BMI160_ERROR_REG_ADDR   UINT8_C(0x02)
#define BMI160_AUX_DATA_ADDR    UINT8_C(0x04)
#define BMI160_GYRO_DATA_ADDR   UINT8_C(0x0C)
#define BMI160_ACCEL_DATA_ADDR    UINT8_C(0x12)
#define BMI160_STATUS_ADDR    UINT8_C(0x1B)
#define BMI160_INT_STATUS_ADDR    UINT8_C(0x1C)
#define BMI160_FIFO_LENGTH_ADDR   UINT8_C(0x22)
#define BMI160_FIFO_DATA_ADDR   UINT8_C(0x24)
#define BMI160_ACCEL_CONFIG_ADDR  UINT8_C(0x40)
#define BMI160_ACCEL_RANGE_ADDR   UINT8_C(0x41)
#define BMI160_GYRO_CONFIG_ADDR   UINT8_C(0x42)
#define BMI160_GYRO_RANGE_ADDR    UINT8_C(0x43)
#define BMI160_AUX_ODR_ADDR   UINT8_C(0x44)
#define BMI160_FIFO_DOWN_ADDR           UINT8_C(0x45)
#define BMI160_FIFO_CONFIG_0_ADDR       UINT8_C(0x46)
#define BMI160_FIFO_CONFIG_1_ADDR       UINT8_C(0x47)
#define BMI160_AUX_IF_0_ADDR    UINT8_C(0x4B)
#define BMI160_AUX_IF_1_ADDR    UINT8_C(0x4C)
#define BMI160_AUX_IF_2_ADDR    UINT8_C(0x4D)
#define BMI160_AUX_IF_3_ADDR    UINT8_C(0x4E)
#define BMI160_AUX_IF_4_ADDR    UINT8_C(0x4F)
#define BMI160_INT_ENABLE_0_ADDR         UINT8_C(0x50)
#define BMI160_INT_ENABLE_1_ADDR         UINT8_C(0x51)
#define BMI160_INT_ENABLE_2_ADDR         UINT8_C(0x52)
#define BMI160_INT_OUT_CTRL_ADDR         UINT8_C(0x53)
#define BMI160_INT_LATCH_ADDR            UINT8_C(0x54)
#define BMI160_INT_MAP_0_ADDR            UINT8_C(0x55)
#define BMI160_INT_MAP_1_ADDR            UINT8_C(0x56)
#define BMI160_INT_MAP_2_ADDR            UINT8_C(0x57)
#define BMI160_INT_DATA_0_ADDR           UINT8_C(0x58)
#define BMI160_INT_DATA_1_ADDR           UINT8_C(0x59)
#define BMI160_INT_LOWHIGH_0_ADDR        UINT8_C(0x5A)
#define BMI160_INT_LOWHIGH_1_ADDR        UINT8_C(0x5B)
#define BMI160_INT_LOWHIGH_2_ADDR        UINT8_C(0x5C)
#define BMI160_INT_LOWHIGH_3_ADDR        UINT8_C(0x5D)
#define BMI160_INT_LOWHIGH_4_ADDR        UINT8_C(0x5E)
#define BMI160_INT_MOTION_0_ADDR         UINT8_C(0x5F)
#define BMI160_INT_MOTION_1_ADDR         UINT8_C(0x60)
#define BMI160_INT_MOTION_2_ADDR         UINT8_C(0x61)
#define BMI160_INT_MOTION_3_ADDR         UINT8_C(0x62)
#define BMI160_INT_TAP_0_ADDR            UINT8_C(0x63)
#define BMI160_INT_TAP_1_ADDR            UINT8_C(0x64)
#define BMI160_INT_ORIENT_0_ADDR         UINT8_C(0x65)
#define BMI160_INT_ORIENT_1_ADDR         UINT8_C(0x66)
#define BMI160_INT_FLAT_0_ADDR           UINT8_C(0x67)
#define BMI160_INT_FLAT_1_ADDR           UINT8_C(0x68)
#define BMI160_FOC_CONF_ADDR             UINT8_C(0x69)
#define BMI160_CONF_ADDR                 UINT8_C(0x6A)

#define BMI160_IF_CONF_ADDR    UINT8_C(0x6B)
#define BMI160_SELF_TEST_ADDR    UINT8_C(0x6D)
#define BMI160_OFFSET_ADDR     UINT8_C(0x71)
#define BMI160_OFFSET_CONF_ADDR    UINT8_C(0x77)
#define BMI160_INT_STEP_CNT_0_ADDR   UINT8_C(0x78)
#define BMI160_INT_STEP_CONFIG_0_ADDR    UINT8_C(0x7A)
#define BMI160_INT_STEP_CONFIG_1_ADDR    UINT8_C(0x7B)
#define BMI160_COMMAND_REG_ADDR    UINT8_C(0x7E)
#define BMI160_SPI_COMM_TEST_ADDR        UINT8_C(0x7F)
#define BMI160_INTL_PULLUP_CONF_ADDR   UINT8_C(0x85)

/** Error code definitions */
#define BMI160_OK                         INT8_C(0)
#define BMI160_E_NULL_PTR                 INT8_C(-1)
#define BMI160_E_COM_FAIL                 INT8_C(-2)
#define BMI160_E_DEV_NOT_FOUND            INT8_C(-3)
#define BMI160_E_OUT_OF_RANGE             INT8_C(-4)
#define BMI160_E_INVALID_INPUT            INT8_C(-5)
#define BMI160_E_ACCEL_ODR_BW_INVALID   INT8_C(-6)
#define BMI160_E_GYRO_ODR_BW_INVALID    INT8_C(-7)
#define BMI160_E_LWP_PRE_FLTR_INT_INVALID INT8_C(-8)
#define BMI160_E_LWP_PRE_FLTR_INVALID   INT8_C(-9)
#define BMI160_E_AUX_NOT_FOUND      INT8_C(-10)
#define BMI160_FOC_FAILURE      INT8_C(-11)
#define BMI160_ERR_CHOOSE      INT8_C(-12)

/** name API warning codes */
#define BMI160_W_GYRO_SELF_TEST_FAIL  INT8_C(1)
#define BMI160_W_ACCEl_SELF_TEST_FAIL INT8_C(2)

/** BMI160 unique chip identifier */
#define BMI160_CHIP_ID                   UINT8_C(0xD1)

/** Soft reset command */
#define BMI160_SOFT_RESET_CMD            UINT8_C(0xb6)
#define BMI160_SOFT_RESET_DELAY_MS       UINT8_C(15)
/** Start FOC command */
#define BMI160_START_FOC_CMD            UINT8_C(0x03)
/** NVM backup enabling command */
#define BMI160_NVM_BACKUP_EN    UINT8_C(0xA0)

/* Delay in ms settings */
#define BMI160_ACCEL_DELAY_MS            UINT8_C(5)
#define BMI160_GYRO_DELAY_MS             UINT8_C(81)
#define BMI160_ONE_MS_DELAY              UINT8_C(1)
#define BMI160_AUX_COM_DELAY     UINT8_C(10)
#define BMI160_GYRO_SELF_TEST_DELAY  UINT8_C(20)
#define BMI160_ACCEL_SELF_TEST_DELAY   UINT8_C(50)

/** Self test configurations */
#define BMI160_ACCEL_SELF_TEST_CONFIG   UINT8_C(0x2C)
#define BMI160_ACCEL_SELF_TEST_POSITIVE_EN  UINT8_C(0x0D)
#define BMI160_ACCEL_SELF_TEST_NEGATIVE_EN  UINT8_C(0x09)
#define BMI160_ACCEL_SELF_TEST_LIMIT    UINT16_C(8192)

/** Power mode settings */
/* Accel power mode */
#define BMI160_ACCEL_NORMAL_MODE         UINT8_C(0x11)
#define BMI160_ACCEL_LOWPOWER_MODE       UINT8_C(0x12)
#define BMI160_ACCEL_SUSPEND_MODE        UINT8_C(0x10)

/* Gyro power mode */
#define BMI160_GYRO_SUSPEND_MODE         UINT8_C(0x14)
#define BMI160_GYRO_NORMAL_MODE          UINT8_C(0x15)
#define BMI160_GYRO_FASTSTARTUP_MODE     UINT8_C(0x17)

/* Aux power mode */
#define BMI160_AUX_SUSPEND_MODE   UINT8_C(0x18)
#define BMI160_AUX_NORMAL_MODE    UINT8_C(0x19)
#define BMI160_AUX_LOWPOWER_MODE  UINT8_C(0x1A)

/** Range settings */
/* Accel Range */
#define BMI160_ACCEL_RANGE_2G            UINT8_C(0x03)
#define BMI160_ACCEL_RANGE_4G            UINT8_C(0x05)
#define BMI160_ACCEL_RANGE_8G            UINT8_C(0x08)
#define BMI160_ACCEL_RANGE_16G           UINT8_C(0x0C)

/* Gyro Range */
#define BMI160_GYRO_RANGE_2000_DPS       UINT8_C(0x00)
#define BMI160_GYRO_RANGE_1000_DPS       UINT8_C(0x01)
#define BMI160_GYRO_RANGE_500_DPS        UINT8_C(0x02)
#define BMI160_GYRO_RANGE_250_DPS        UINT8_C(0x03)
#define BMI160_GYRO_RANGE_125_DPS        UINT8_C(0x04)

/** Bandwidth settings */
/* Accel Bandwidth */
#define BMI160_ACCEL_BW_OSR4_AVG1        UINT8_C(0x00)
#define BMI160_ACCEL_BW_OSR2_AVG2        UINT8_C(0x01)
#define BMI160_ACCEL_BW_NORMAL_AVG4      UINT8_C(0x02)
#define BMI160_ACCEL_BW_RES_AVG8         UINT8_C(0x03)
#define BMI160_ACCEL_BW_RES_AVG16        UINT8_C(0x04)
#define BMI160_ACCEL_BW_RES_AVG32        UINT8_C(0x05)
#define BMI160_ACCEL_BW_RES_AVG64        UINT8_C(0x06)
#define BMI160_ACCEL_BW_RES_AVG128       UINT8_C(0x07)

#define BMI160_GYRO_BW_OSR4_MODE         UINT8_C(0x00)
#define BMI160_GYRO_BW_OSR2_MODE         UINT8_C(0x01)
#define BMI160_GYRO_BW_NORMAL_MODE       UINT8_C(0x02)

/* Output Data Rate settings */
/* Accel Output data rate */
#define BMI160_ACCEL_ODR_RESERVED        UINT8_C(0x00)
#define BMI160_ACCEL_ODR_0_78HZ          UINT8_C(0x01)
#define BMI160_ACCEL_ODR_1_56HZ          UINT8_C(0x02)
#define BMI160_ACCEL_ODR_3_12HZ          UINT8_C(0x03)
#define BMI160_ACCEL_ODR_6_25HZ          UINT8_C(0x04)
#define BMI160_ACCEL_ODR_12_5HZ          UINT8_C(0x05)
#define BMI160_ACCEL_ODR_25HZ            UINT8_C(0x06)
#define BMI160_ACCEL_ODR_50HZ            UINT8_C(0x07)
#define BMI160_ACCEL_ODR_100HZ           UINT8_C(0x08)
#define BMI160_ACCEL_ODR_200HZ           UINT8_C(0x09)
#define BMI160_ACCEL_ODR_400HZ           UINT8_C(0x0A)
#define BMI160_ACCEL_ODR_800HZ           UINT8_C(0x0B)
#define BMI160_ACCEL_ODR_1600HZ          UINT8_C(0x0C)
#define BMI160_ACCEL_ODR_RESERVED0       UINT8_C(0x0D)
#define BMI160_ACCEL_ODR_RESERVED1       UINT8_C(0x0E)
#define BMI160_ACCEL_ODR_RESERVED2       UINT8_C(0x0F)

/* Gyro Output data rate */
#define BMI160_GYRO_ODR_RESERVED         UINT8_C(0x00)
#define BMI160_GYRO_ODR_25HZ             UINT8_C(0x06)
#define BMI160_GYRO_ODR_50HZ             UINT8_C(0x07)
#define BMI160_GYRO_ODR_100HZ            UINT8_C(0x08)
#define BMI160_GYRO_ODR_200HZ            UINT8_C(0x09)
#define BMI160_GYRO_ODR_400HZ            UINT8_C(0x0A)
#define BMI160_GYRO_ODR_800HZ            UINT8_C(0x0B)
#define BMI160_GYRO_ODR_1600HZ           UINT8_C(0x0C)
#define BMI160_GYRO_ODR_3200HZ           UINT8_C(0x0D)

/* Auxiliary sensor Output data rate */
#define BMI160_AUX_ODR_RESERVED        UINT8_C(0x00)
#define BMI160_AUX_ODR_0_78HZ          UINT8_C(0x01)
#define BMI160_AUX_ODR_1_56HZ          UINT8_C(0x02)
#define BMI160_AUX_ODR_3_12HZ          UINT8_C(0x03)
#define BMI160_AUX_ODR_6_25HZ          UINT8_C(0x04)
#define BMI160_AUX_ODR_12_5HZ          UINT8_C(0x05)
#define BMI160_AUX_ODR_25HZ            UINT8_C(0x06)
#define BMI160_AUX_ODR_50HZ            UINT8_C(0x07)
#define BMI160_AUX_ODR_100HZ           UINT8_C(0x08)
#define BMI160_AUX_ODR_200HZ           UINT8_C(0x09)
#define BMI160_AUX_ODR_400HZ           UINT8_C(0x0A)
#define BMI160_AUX_ODR_800HZ           UINT8_C(0x0B)

/* Maximum limits definition */
#define BMI160_ACCEL_ODR_MAX             UINT8_C(15)
#define BMI160_ACCEL_BW_MAX              UINT8_C(2)
#define BMI160_ACCEL_RANGE_MAX           UINT8_C(12)
#define BMI160_GYRO_ODR_MAX              UINT8_C(13)
#define BMI160_GYRO_BW_MAX               UINT8_C(2)
#define BMI160_GYRO_RANGE_MAX            UINT8_C(4)

/** FIFO_CONFIG Definitions */
#define BMI160_FIFO_TIME_ENABLE          UINT8_C(0x02)
#define BMI160_FIFO_TAG_INT2_ENABLE      UINT8_C(0x04)
#define BMI160_FIFO_TAG_INT1_ENABLE      UINT8_C(0x08)
#define BMI160_FIFO_HEAD_ENABLE          UINT8_C(0x10)
#define BMI160_FIFO_M_ENABLE             UINT8_C(0x20)
#define BMI160_FIFO_A_ENABLE             UINT8_C(0x40)
#define BMI160_FIFO_M_A_ENABLE           UINT8_C(0x60)
#define BMI160_FIFO_G_ENABLE             UINT8_C(0x80)
#define BMI160_FIFO_M_G_ENABLE           UINT8_C(0xA0)
#define BMI160_FIFO_G_A_ENABLE           UINT8_C(0xC0)
#define BMI160_FIFO_M_G_A_ENABLE         UINT8_C(0xE0)


/* Accel, gyro and aux. sensor length and also their combined
 * length definitions in FIFO */
#define BMI160_FIFO_G_LENGTH             UINT8_C(6)
#define BMI160_FIFO_A_LENGTH             UINT8_C(6)
#define BMI160_FIFO_M_LENGTH             UINT8_C(8)
#define BMI160_FIFO_GA_LENGTH            UINT8_C(12)
#define BMI160_FIFO_MA_LENGTH            UINT8_C(14)
#define BMI160_FIFO_MG_LENGTH            UINT8_C(14)
#define BMI160_FIFO_MGA_LENGTH           UINT8_C(20)


/** FIFO Header Data definitions */
#define BMI160_FIFO_HEAD_SKIP_FRAME      UINT8_C(0x40)
#define BMI160_FIFO_HEAD_SENSOR_TIME     UINT8_C(0x44)
#define BMI160_FIFO_HEAD_INPUT_CONFIG    UINT8_C(0x48)
#define BMI160_FIFO_HEAD_OVER_READ       UINT8_C(0x80)
#define BMI160_FIFO_HEAD_A               UINT8_C(0x84)
#define BMI160_FIFO_HEAD_G               UINT8_C(0x88)
#define BMI160_FIFO_HEAD_G_A             UINT8_C(0x8C)
#define BMI160_FIFO_HEAD_M               UINT8_C(0x90)
#define BMI160_FIFO_HEAD_M_A             UINT8_C(0x94)
#define BMI160_FIFO_HEAD_M_G             UINT8_C(0x98)
#define BMI160_FIFO_HEAD_M_G_A           UINT8_C(0x9C)


/** FIFO sensor time length definitions */
#define BMI160_SENSOR_TIME_LENGTH        UINT8_C(3)


/** FIFO DOWN selection */
/* Accel fifo down-sampling values*/
#define  BMI160_ACCEL_FIFO_DOWN_ZERO     UINT8_C(0x00)
#define  BMI160_ACCEL_FIFO_DOWN_ONE      UINT8_C(0x10)
#define  BMI160_ACCEL_FIFO_DOWN_TWO      UINT8_C(0x20)
#define  BMI160_ACCEL_FIFO_DOWN_THREE    UINT8_C(0x30)
#define  BMI160_ACCEL_FIFO_DOWN_FOUR     UINT8_C(0x40)
#define  BMI160_ACCEL_FIFO_DOWN_FIVE     UINT8_C(0x50)
#define  BMI160_ACCEL_FIFO_DOWN_SIX      UINT8_C(0x60)
#define  BMI160_ACCEL_FIFO_DOWN_SEVEN    UINT8_C(0x70)

/* Gyro fifo down-smapling values*/
#define  BMI160_GYRO_FIFO_DOWN_ZERO      UINT8_C(0x00)
#define  BMI160_GYRO_FIFO_DOWN_ONE       UINT8_C(0x01)
#define  BMI160_GYRO_FIFO_DOWN_TWO       UINT8_C(0x02)
#define  BMI160_GYRO_FIFO_DOWN_THREE     UINT8_C(0x03)
#define  BMI160_GYRO_FIFO_DOWN_FOUR      UINT8_C(0x04)
#define  BMI160_GYRO_FIFO_DOWN_FIVE      UINT8_C(0x05)
#define  BMI160_GYRO_FIFO_DOWN_SIX       UINT8_C(0x06)
#define  BMI160_GYRO_FIFO_DOWN_SEVEN     UINT8_C(0x07)

/* Accel Fifo filter enable*/
#define  BMI160_ACCEL_FIFO_FILT_EN       UINT8_C(0x80)

/* Gyro Fifo filter enable*/
#define  BMI160_GYRO_FIFO_FILT_EN        UINT8_C(0x08)

/** Definitions to check validity of FIFO frames */
#define FIFO_CONFIG_MSB_CHECK            UINT8_C(0x80)
#define FIFO_CONFIG_LSB_CHECK            UINT8_C(0x00)

/*! BMI160 accel FOC configurations */
#define BMI160_FOC_ACCEL_DISABLED        UINT8_C(0x00)
#define BMI160_FOC_ACCEL_POSITIVE_G      UINT8_C(0x01)
#define BMI160_FOC_ACCEL_NEGATIVE_G      UINT8_C(0x02)
#define BMI160_FOC_ACCEL_0G              UINT8_C(0x03)

/** Array Parameter DefinItions */
#define BMI160_SENSOR_TIME_LSB_BYTE      UINT8_C(0)
#define BMI160_SENSOR_TIME_XLSB_BYTE     UINT8_C(1)
#define BMI160_SENSOR_TIME_MSB_BYTE      UINT8_C(2)


/** Interface settings */
#define BMI160_SPI_INTF                  UINT8_C(1)
#define BMI160_I2C_INTF                  UINT8_C(0)
#define BMI160_SPI_RD_MASK               UINT8_C(0x80)
#define BMI160_SPI_WR_MASK               UINT8_C(0x7F)

/* Sensor & time select definition*/
#define BMI160_ACCEL_SEL    UINT8_C(0x01)
#define BMI160_GYRO_SEL     UINT8_C(0x02)
#define BMI160_TIME_SEL     UINT8_C(0x04)

/* Sensor select mask*/
#define BMI160_SEN_SEL_MASK   UINT8_C(0x07)

/* Error code mask */
#define BMI160_ERR_REG_MASK   UINT8_C(0x0F)

/* BMI160 I2C address */
#define BMI160_I2C_ADDR                 UINT8_C(0x68)

/* BMI160 secondary IF address */
#define BMI160_AUX_BMM150_I2C_ADDR    UINT8_C(0x10)

/** BMI160 Length definitions */
#define BMI160_ONE                       UINT8_C(1)
#define BMI160_TWO                       UINT8_C(2)
#define BMI160_THREE                     UINT8_C(3)
#define BMI160_FOUR                      UINT8_C(4)
#define BMI160_FIVE                      UINT8_C(5)

/** BMI160 fifo level Margin */
#define BMI160_FIFO_LEVEL_MARGIN         UINT8_C(16)

/** BMI160 fifo flush Command */
#define BMI160_FIFO_FLUSH_VALUE          UINT8_C(0xB0)

/** BMI160 offset values for xyz axes of accel */
#define BMI160_ACCEL_MIN_OFFSET         INT8_C(-128)
#define BMI160_ACCEL_MAX_OFFSET         INT8_C(127)

/** BMI160 offset values for xyz axes of gyro */
#define BMI160_GYRO_MIN_OFFSET         INT16_C(-512)
#define BMI160_GYRO_MAX_OFFSET         INT16_C(511)

/** BMI160 fifo full interrupt position and mask */
#define BMI160_FIFO_FULL_INT_POS  UINT8_C(5)
#define BMI160_FIFO_FULL_INT_MSK  UINT8_C(0x20)
#define BMI160_FIFO_WTM_INT_POS   UINT8_C(6)
#define BMI160_FIFO_WTM_INT_MSK   UINT8_C(0x40)

#define BMI160_FIFO_FULL_INT_PIN1_POS UINT8_C(5)
#define BMI160_FIFO_FULL_INT_PIN1_MSK UINT8_C(0x20)
#define BMI160_FIFO_FULL_INT_PIN2_POS UINT8_C(1)
#define BMI160_FIFO_FULL_INT_PIN2_MSK UINT8_C(0x02)

#define BMI160_FIFO_WTM_INT_PIN1_POS  UINT8_C(6)
#define BMI160_FIFO_WTM_INT_PIN1_MSK  UINT8_C(0x40)
#define BMI160_FIFO_WTM_INT_PIN2_POS  UINT8_C(2)
#define BMI160_FIFO_WTM_INT_PIN2_MSK  UINT8_C(0x04)

#define BMI160_MANUAL_MODE_EN_POS UINT8_C(7)
#define BMI160_MANUAL_MODE_EN_MSK UINT8_C(0x80)
#define BMI160_AUX_READ_BURST_POS UINT8_C(0)
#define BMI160_AUX_READ_BURST_MSK UINT8_C(0x03)

#define BMI160_GYRO_SELF_TEST_POS UINT8_C(4)
#define BMI160_GYRO_SELF_TEST_MSK UINT8_C(0x10)
#define BMI160_GYRO_SELF_TEST_STATUS_POS  UINT8_C(1)
#define BMI160_GYRO_SELF_TEST_STATUS_MSK  UINT8_C(0x02)

#define BMI160_GYRO_FOC_EN_POS  UINT8_C(6)
#define BMI160_GYRO_FOC_EN_MSK  UINT8_C(0x40)

#define BMI160_ACCEL_FOC_X_CONF_POS UINT8_C(4)
#define BMI160_ACCEL_FOC_X_CONF_MSK UINT8_C(0x30)

#define BMI160_ACCEL_FOC_Y_CONF_POS UINT8_C(2)
#define BMI160_ACCEL_FOC_Y_CONF_MSK UINT8_C(0x0C)

#define BMI160_ACCEL_FOC_Z_CONF_MSK UINT8_C(0x03)

#define BMI160_FOC_STATUS_POS UINT8_C(3)
#define BMI160_FOC_STATUS_MSK UINT8_C(0x08)

#define BMI160_GYRO_OFFSET_X_MSK  UINT8_C(0x03)

#define BMI160_GYRO_OFFSET_Y_POS  UINT8_C(2)
#define BMI160_GYRO_OFFSET_Y_MSK  UINT8_C(0x0C)

#define BMI160_GYRO_OFFSET_Z_POS  UINT8_C(4)
#define BMI160_GYRO_OFFSET_Z_MSK  UINT8_C(0x30)

#define BMI160_GYRO_OFFSET_EN_POS UINT8_C(7)
#define BMI160_GYRO_OFFSET_EN_MSK UINT8_C(0x80)

#define BMI160_ACCEL_OFFSET_EN_POS  UINT8_C(6)
#define BMI160_ACCEL_OFFSET_EN_MSK  UINT8_C(0x40)


#define BMI160_GYRO_OFFSET_POS          UINT16_C(8)
#define BMI160_GYRO_OFFSET_MSK          UINT16_C(0x0300)

#define BMI160_NVM_UPDATE_POS         UINT8_C(1)
#define BMI160_NVM_UPDATE_MSK         UINT8_C(0x02)

#define BMI160_NVM_STATUS_POS         UINT8_C(4)
#define BMI160_NVM_STATUS_MSK         UINT8_C(0x10)

/* BIT SLICE GET AND SET FUNCTIONS */
#define  BMI160_GET_BITS(regvar, bitname)\
    ((regvar & bitname##_MSK) >> bitname##_POS)
#define BMI160_SET_BITS(regvar, bitname, val)\
    ((regvar & ~bitname##_MSK) | \
    ((val<<bitname##_POS)&bitname##_MSK))

#define BMI160_SET_BITS_POS_0(reg_data, bitname, data) \
        ((reg_data & ~(bitname##_MSK)) | \
        (data & bitname##_MSK))

#define BMI160_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/**\name UTILITY MACROS */
#define BMI160_SET_LOW_BYTE     UINT16_C(0x00FF)
#define BMI160_SET_HIGH_BYTE    UINT16_C(0xFF00)

#define BMI160_GET_LSB(var) (uint8_t)(var & BMI160_SET_LOW_BYTE)
#define BMI160_GET_MSB(var) (uint8_t)((var & BMI160_SET_HIGH_BYTE) >> 8)

/**
  * @struct bmi160FifoFrame
  * @brief This structure holds the information for usage of FIFO by the user.
  */
struct bmi160FifoFrame {
  uint8_t *data;             /**< Data buffer of user defined length is to be mapped here */
  uint16_t length;           /**< While calling the API  "bmi160_get_fifo_data" , length stores number of bytes in FIFO to be read (specified by user as input) and after execution of the API ,number of FIFO data bytes available is provided as an output to user*/
  uint8_t fifoTimeEnable;    /**< FIFO time enable */
  uint8_t fifoHeaderEnable;  /**< Enabling of the FIFO header to stream in header mode */
  uint8_t fifoDataEnable;    /**< Streaming of the Accelerometer, Gyroscope sensor data or both in FIFO */
  uint16_t accelByteStartIdx;/**< Will be equal to length when no more frames are there to parse */
  uint16_t gyroByteStartIdx; /**< Will be equal to length when no more frames are there to parse */
  uint16_t auxByteStartIdx;  /**< Will be equal to length when no more frames are there to parse */
  uint32_t sensorTime;       /**< Value of FIFO sensor time time */
  uint8_t skippedFrameCount; /**< Value of Skipped frame counts */
};

/**
 * @enum eBmi160AnySigMotionActiveInterruptState
 * @brief bmi160 active state of any & sig motion interrupt.
 */
enum eBmi160AnySigMotionActiveInterruptState { 
  eBmi160BothAnySigMotionDisabled = -1, /**< Both any & sig motion are disabled */
  eBmi160AnyMotionEnabled,              /**< Any-motion selected */
  eBmi160SigMotionEnabled               /**< Sig-motion selected */
};

/**
 * @enum eBmi160SelectSensor
 * @brief bmi160 sensor select structure
 */
enum eBmi160SelectSensor {
  eBmi160AccelOnly = 1,
  eBmi160GyroOnly,
  eBmi160BothAccelAndGyro
};

/**
 * @struct bmi160Cfg
 * @brief bmi160 sensor configuration structure
 */
struct bmi160Cfg {
  uint8_t power;  /**< power mode */
  uint8_t odr;    /**< output data rate */
  uint8_t range;  /**< range */
  uint8_t bw;     /**< bandwidth */
};

/**
 * @brief Aux sensor configuration structure
 */
struct bmi160AuxCfg {
  uint8_t auxSensorEnable : 1;  /**< Aux sensor, 1 - enable 0 - disable */
  uint8_t manualEnable : 1;     /**< Aux manual/auto mode status */
  uint8_t auxRdBurstLen : 2;    /**< Aux read burst length */
  uint8_t auxOdr :4;            /**< output data rate */
  uint8_t auxI2cAddr;           /**< i2c addr of auxiliary sensor */
};

/* type definitions */
typedef int8_t (*bmi160ComFptrT)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bmi160DelayFptrT)(uint32_t period);

struct bmi160Dev {
  uint8_t chipId;   /**< Chip Id */
  uint8_t id;       /**< Device Id */
  uint8_t interface;/**< 0 - I2C , 1 - SPI Interface */
  enum eBmi160AnySigMotionActiveInterruptState any_sig_sel;/**< Hold active interrupts status for any and sig motion 0 - Any-motion enable, 1 - Sig-motion enable,  -1 neither any-motion nor sig-motion selected */
  struct bmi160Cfg accelCfg;                               /**< Structure to configure Accel sensor */
  struct bmi160Cfg prevAccelCfg;/**< Structure to hold previous/old accel config parameters.This is used at driver level to prevent overwriting of samedata, hence user does not change it in the code */
  struct bmi160Cfg gyroCfg;     /**< Structure to configure Gyro sensor */
  struct bmi160Cfg prevGyroCfg; /**< Structure to hold previous/old gyro config parameters.This is used at driver level to prevent overwriting of same data, hence user does not change it in the code */
  struct bmi160AuxCfg auxCfg;   /**< Structure to configure the auxiliary sensor */
  struct bmi160AuxCfg prevAuxCfg;/**< Structure to hold previous/old aux config parameters.This is used at driver level to prevent overwriting of samedata, hence user does not change it in the code */
  struct bmi160FifoFrame *fifo; /**< FIFO related configurations */
  bmi160ComFptrT read;          /**< Read function pointer */
  bmi160ComFptrT write;         /**< Write function pointer */
  bmi160DelayFptrT delayMs;     /**<  Delay function pointer */
};

/**
 * @brief bmi160 sensor data structure which comprises of accel data
 */
struct bmi160SensorData {
  int16_t x;           /**< X-axis sensor data */
  int16_t y;           /**< Y-axis sensor data */
  int16_t z;           /**< Z-axis sensor data */
  uint32_t sensortime; /**< sensor time */
};

enum bmi160IntChannel {
  BMI160_INT_CHANNEL_NONE, /**< Un-map both channels */
  BMI160_INT_CHANNEL_1,    /**< interrupt Channel 1 */
  BMI160_INT_CHANNEL_2,    /**< interrupt Channel 2 */
  BMI160_INT_CHANNEL_BOTH  /**< Map both channels */
};

enum bmi160IntTypes {
  BMI160_ACC_ANY_MOTION_INT,         /**< Slope/Any-motion interrupt */
  BMI160_ACC_SIG_MOTION_INT,         /**< Significant motion interrupt */
  BMI160_STEP_DETECT_INT,            /**< Step detector interrupt */
  BMI160_ACC_DOUBLE_TAP_INT,         /**< double tap interrupt */
  BMI160_ACC_SINGLE_TAP_INT,         /**< single tap interrupt */
  BMI160_ACC_ORIENT_INT,             /**< orientation interrupt */
  BMI160_ACC_FLAT_INT,               /**< flat interrupt */
  BMI160_ACC_HIGH_G_INT,             /**< high-g interrupt */
  BMI160_ACC_LOW_G_INT,              /**< low-g interrupt */
  BMI160_ACC_SLOW_NO_MOTION_INT,     /**< slow/no-motion interrupt */
  BMI160_ACC_GYRO_DATA_RDY_INT,      /**< data ready interrupt  */
  BMI160_ACC_GYRO_FIFO_FULL_INT,     /**< fifo full interrupt */
  BMI160_ACC_GYRO_FIFO_WATERMARK_INT /**< fifo watermark interrupt */
};

struct bmi160IntPinSettg {
#if LITTLE_ENDIAN == 1
  
  uint16_t outputEn :1;   /**< To enable either INT1 or INT2 pin as output. 0- output disabled ,1- output enabled */
  uint16_t outputMode :1; /**< 0 - push-pull 1- open drain,only valid if outputEn is set 1 */
  uint16_t outputType :1; /**< 0 - active low , 1 - active high level. if outputEn is 1,this applies to interrupts,else PMU_trigger */
  uint16_t edgeCtrl :1;   /**< 0 - level trigger , 1 - edge trigger  */
  uint16_t inputEn :1;    /**< To enable either INT1 or INT2 pin as input. 0 - input disabled ,1 - input enabled */
  uint16_t latchDur :4;   /**< latch duration*/
#elif BIG_ENDIAN == 1
  uint16_t latchDur : 4;  /**< latch duration*/
  uint16_t inputEn : 1;   /**< Latched,non-latched or temporary interrupt modes */
  uint16_t edgeCtrl : 1;  /**< 1 - edge trigger, 0 - level trigger */
  uint16_t outputType : 1;/**< 0 - active low , 1 - active high level. if outputEn is 1,this applies to interrupts,else PMU_trigger */
  uint16_t outputMode : 1;/**< 0 - push-pull , 1 - open drain,only valid if outputEn is set 1 */
  uint16_t outputEn : 1;  /**< To enable either INT1 or INT2 pin as output. 0 - output disabled , 1 - output enabled */
#endif
};

struct bmi160AccTapIntCfg {
#if LITTLE_ENDIAN == 1
  uint16_t tapThr :5;     /**< tap threshold */
  uint16_t tapShock :1;   /**< tap shock */
  uint16_t tapQuiet :1;   /**< tap quiet */
  uint16_t tapDur :3;     /**< tap duration */
  uint16_t tapDataSrc :1; /**< data source 0- filter & 1 pre-filter*/
  uint16_t tapEn :1;      /**< tap enable, 1 - enable, 0 - disable */
#elif BIG_ENDIAN == 1
  uint16_t tapEn :1;     /**< tap enable, 1 - enable, 0 - disable */
  uint16_t tapDataSrc :1;/**< data source 0- filter & 1 pre-filter*/
  uint16_t tapDur : 3;   /**< tap duration */
  uint16_t tapQuiet : 1; /**< tap quiet */
  uint16_t tapShock : 1; /**< tap shock */
  uint16_t tapThr : 5;   /**< tap threshold */
#endif
};

struct bmi160AccAnyMotIntCfg {
#if LITTLE_ENDIAN == 1
  uint8_t anymotionEn :1;     /**< 1 any-motion enable, 0 - any-motion disable */
  uint8_t anymotionX :1;      /**< slope interrupt x, 1 - enable, 0 - disable */
  uint8_t anymotionY :1;      /**< slope interrupt y, 1 - enable, 0 - disable */
  uint8_t anymotionZ :1;      /**< slope interrupt z, 1 - enable, 0 - disable */
  uint8_t anymotionDur :2;    /**< slope duration */
  uint8_t anymotionDataSrc :1;/**< data source 0- filter & 1 pre-filter*/
  uint8_t anymotionThr;       /**< slope threshold */
#elif BIG_ENDIAN == 1
  
  uint8_t anymotionThr;       /**< slope threshold */
  uint8_t anymotionDataSrc :1;/**< data source 0- filter & 1 pre-filter*/
  uint8_t anymotionDur : 2;   /**< slope duration */
  uint8_t anymotionZ : 1;     /**< slope interrupt z, 1 - enable, 0 - disable */
  uint8_t anymotionY : 1;     /**< slope interrupt y, 1 - enable, 0 - disable */
  uint8_t anymotionX : 1;     /**< slope interrupt x, 1 - enable, 0 - disable */
  uint8_t anymotionEn :1;     /**< 1 any-motion enable, 0 - any-motion disable */
#endif
};

struct bmi160AccSigMotIntCfg {
#if LITTLE_ENDIAN == 1
  uint8_t sigMotSkip :2;  /**< skip time of sig-motion interrupt */
  uint8_t sigMotProof :2; /**< proof time of sig-motion interrupt */
  uint8_t sigDataSrc :1;  /**< data source 0- filter & 1 pre-filter*/
  uint8_t sigEn :1;       /**< 1 - enable sig, 0 - disable sig & enable anymotion */
  uint8_t sigMotThres;    /**< sig-motion threshold */
#elif BIG_ENDIAN == 1
  
  uint8_t sigMotThres;    /**< sig-motion threshold */
  uint8_t sigEn :1;       /**< 1 - enable sig, 0 - disable sig & enable anymotion */
  uint8_t sigDataSrc :1;  /**< data source 0- filter & 1 pre-filter*/
  uint8_t sigMotProof : 2;/**< proof time of sig-motion interrupt */
  uint8_t sigMotSkip : 2; /**< skip time of sig-motion interrupt */
#endif
};

struct bmi160AccStepDetectIntCfg {
#if LITTLE_ENDIAN == 1
  uint16_t stepDetectorEn :1;  /**< 1- step detector enable, 0- step detector disable */
  uint16_t minThreshold :2;    /**< minimum threshold */
  uint16_t steptimeMin :3;     /**< minimal detectable step time */
  uint16_t stepDetectorMode :2;/**< enable step counter mode setting */
  uint16_t stepMinBuf :3;      /**< minimum step buffer size*/
#elif BIG_ENDIAN == 1
  uint16_t stepMinBuf :3;       /**< minimum step buffer size*/
  uint16_t stepDetectorMode : 2;/**< enable step counter mode setting */
  uint16_t steptimeMin : 3;     /**< minimal detectable step time */
  uint16_t minThreshold : 2;    /**< minimum threshold */
  uint16_t stepDetectorEn :1;   /**< 1- step detector enable, 0- step detector disable */
#endif
};

struct bmi160AccNoMotionIntCfg {
#if LITTLE_ENDIAN == 1
  uint16_t noMotionX :1;  /**< no motion interrupt x */
  uint16_t noMotionY :1;  /**< no motion interrupt y */
  uint16_t noMotionZ :1;  /**< no motion interrupt z */
  uint16_t noMotionDur :6;/**< no motion duration */
  uint16_t noMotionSel :1;/**< no motion sel , 1 - enable no-motion ,0- enable slow-motion */
  uint16_t noMotionSrc :1;/**< data source 0- filter & 1 pre-filter*/
  uint8_t noMotionThres;  /**< no motion threshold */
#elif BIG_ENDIAN == 1
  uint8_t noMotionThres;   /**< no motion threshold */
  uint16_t noMotionSrc :1; /**< data source 0- filter & 1 pre-filter*/
  uint16_t noMotionSel : 1;/**< no motion sel , 1 - enable no-motion ,0- enable slow-motion */
  uint16_t noMotionDur : 6;/**< no motion duration */
  uint16_t noMotionZ :1;   /**< no motion interrupt z */
  uint16_t noMotionY :1;   /**< no motion interrupt y */
  uint16_t noMotionX :1;   /**< no motion interrupt x */
#endif
};

struct bmi160AccOrientIntCfg {
#if LITTLE_ENDIAN == 1
  uint16_t orientMode :2;    /**< thresholds for switching between the different orientations */
  uint16_t orientBlocking :2;/**< blocking_mode */
  uint16_t orientHyst :4;    /**< Orientation interrupt hysteresis */
  uint16_t orientTheta :6;   /**< Orientation interrupt theta */
  uint16_t orientUdEn :1;    /**< Enable/disable Orientation interrupt */
  uint16_t axesEx :1;        /**< exchange x- and z-axis in algorithm ,0 - z, 1 - x */
  uint8_t orientEn :1;       /**< 1 - orient enable, 0 - orient disable */
#elif BIG_ENDIAN == 1
  uint8_t orientEn :1;        /**< 1 - orient enable, 0 - orient disable */
  uint16_t axesEx : 1;        /**< exchange x- and z-axis in algorithm ,0 - z, 1 - x */
  uint16_t orientUdEn : 1;    /**< Enable/disable Orientation interrupt */
  uint16_t orientTheta : 6;   /**< Orientation interrupt theta */
  uint16_t orientHyst : 4;    /**< Orientation interrupt hysteresis */
  uint16_t orientBlocking : 2;/**< blocking_mode */
  uint16_t orientMode : 2;    /**< thresholds for switching between the different orientations */
#endif
};

struct bmi160AccFlatDetectIntCfg {
#if LITTLE_ENDIAN == 1
  
  uint16_t flatTheta :6;    /**< flat threshold */
  uint16_t flatHy :3;       /**< flat interrupt hysteresis */
  uint16_t flatHoldTime :2; /**< delay time for which the flat value must remain stable for the flat interrupt to be generated */
  uint16_t flatEn :1;       /**< 1 - flat enable, 0 - flat disable */
#elif BIG_ENDIAN == 1
  uint16_t flatEn :1;         /**< 1 - flat enable, 0 - flat disable */
  uint16_t flatHoldTime : 2;  /**< delay time for which the flat value must remain stable for the flat interrupt to be generated */
  uint16_t flatHy : 3;        /**< flat interrupt hysteresis */
  uint16_t flatTheta : 6;     /**< flat threshold */
#endif
};

struct bmi160AccLowGIntCfg {
#if LITTLE_ENDIAN == 1
  uint8_t lowDur;       /**< low-g interrupt trigger delay */
  uint8_t lowThres;     /**< low-g interrupt trigger threshold */
  uint8_t lowHyst :2;   /**< hysteresis of low-g interrupt */
  uint8_t lowMode :1;   /**< 0 - single-axis mode ,1 - axis-summing mode */
  uint8_t lowDataSrc :1;/**< data source 0- filter & 1 pre-filter */
  uint8_t lowEn :1;     /**< 1 - enable low-g, 0 - disable low-g */
#elif BIG_ENDIAN == 1
  uint8_t lowEn :1;     /**< 1 - enable low-g, 0 - disable low-g */
  uint8_t lowDataSrc :1;/**< data source 0- filter & 1 pre-filter */
  uint8_t lowMode : 1;  /**< 0 - single-axis mode ,1 - axis-summing mode */
  uint8_t lowHyst : 2;  /**< hysteresis of low-g interrupt */
  uint8_t lowThres;     /**< low-g interrupt trigger threshold */
  uint8_t lowDur;       /**< low-g interrupt trigger delay */
#endif
};

struct bmi160AccHighGIntCfg {
#if LITTLE_ENDIAN == 1
  
  uint8_t high_g_x :1;    /**< High-g interrupt x, 1 - enable, 0 - disable */
  uint8_t high_g_y :1;    /**< High-g interrupt y, 1 - enable, 0 - disable */
  uint8_t high_g_z :1;    /**< High-g interrupt z, 1 - enable, 0 - disable */
  uint8_t highHy :2;      /**< High-g hysteresis  */
  uint8_t highDataSrc :1; /**< data source 0- filter & 1 pre-filter */
  uint8_t highThres;      /**< High-g threshold */
  uint8_t highDur;        /**< High-g duration */
#elif BIG_ENDIAN == 1
  uint8_t highDur;       /**< High-g duration */
  uint8_t highThres;     /**< High-g threshold */
  uint8_t highDataSrc :1;/**< data source 0- filter & 1 pre-filter */
  uint8_t highHy : 2;    /**< High-g hysteresis  */
  uint8_t high_g_z : 1;  /**< High-g interrupt z, 1 - enable, 0 - disable */
  uint8_t high_g_y : 1;  /**< High-g interrupt y, 1 - enable, 0 - disable */
  uint8_t high_g_x : 1;  /**< High-g interrupt x, 1 - enable, 0 - disable */
#endif
};

union bmi160IntTypeCfg {
  struct bmi160AccTapIntCfg accTapInt;               /**< Tap interrupt structure */
  struct bmi160AccAnyMotIntCfg accAnyMotionInt;      /**< Slope interrupt structure */
  struct bmi160AccSigMotIntCfg accSigMotionInt;      /**< Significant motion interrupt structure */
  struct bmi160AccStepDetectIntCfg accStepDetectInt; /**< Step detector interrupt structure */
  struct bmi160AccNoMotionIntCfg accNoMotionInt;     /**< No motion interrupt structure */
  struct bmi160AccOrientIntCfg accOrientInt;         /**< Orientation interrupt structure */
  struct bmi160AccFlatDetectIntCfg accFlatInt;       /**< Flat interrupt structure */
  struct bmi160AccLowGIntCfg accLowGInt;             /**< Low-g interrupt structure */
  struct bmi160AccHighGIntCfg accHighGInt;           /**< High-g interrupt structure */
};

struct bmi160IntSettg {
  enum bmi160IntChannel intChannel;    /**< Interrupt channel */
  enum bmi160IntTypes intType;         /**< Select Interrupt */
  struct bmi160IntPinSettg intPinSettg;/**< Structure configuring Interrupt pins */
  union bmi160IntTypeCfg intTypeCfg;   /**< Union configures required interrupt */
  uint8_t fifoFullIntEn :1;            /**< FIFO FULL INT 1-enable, 0-disable */
  uint8_t fifoWTMIntEn :1;             /**< FIFO WTM INT 1-enable, 0-disable */
};

enum bmi160StepDetectMode {
  BMI160_STEP_DETECT_NORMAL,
  BMI160_STEP_DETECT_SENSITIVE,
  BMI160_STEP_DETECT_ROBUST,
  BMI160_STEP_DETECT_USER_DEFINE   /**< Non recommended User defined setting */
};

class DFRobot_BMI160{
public:
  DFRobot_BMI160();
  ~DFRobot_BMI160();	// [MV]
  
  /**
   * @fn I2cInit
   * @brief set the i2c addr and init the i2c.
   * @param i2c_addr  bmi160 i2c addr
   * @n     0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
   * @n     0x69: set I2C address by parameter
   * @return BMI160_OK(0) means success
   */
  int8_t I2cInit(char *path = (char*)"/dev/i2c-1", int8_t i2c_addr = BMI160_I2C_ADDR);
  
  /**
   * @fn getSensorData
   * @brief select mode and save returned data to parameter data.
   * @param type  three type
   * @n     onlyAccel    :   only get the accel data
   * @n     onlyGyro     :   only get the gyro data
   * @n     bothAccelGyro:   get boath accel and gyro data
   * @param data  save returned data to parameter data
   * @return BMI160_OK(0) means succse
   */
  int8_t getSensorData(uint8_t type,int16_t* data);
  
  /**
   * @fn getAccelData
   * @brief get the accel data 
   * @param data pointer to store the accel data
   * @return BMI160_OK(0) means succse
   */
  int8_t getAccelData(int16_t* data);
  
  /**
   * @fn getAccelData
   * @brief get the accel data 
   * @param data pointer to store the accel data
   * @param timestamp pointer to store the timestamp for accel
   * @return BMI160_OK(0) means succse
   */
  int8_t getAccelData(int16_t* data, uint32_t* timestamp);
  
  /**
   * @fn getGyroData
   * @brief get the gyro data 
   * @param data pointer to store the gyro data
   * @return BMI160_OK(0) means succse
   */
  int8_t getGyroData(int16_t* data);
  
  /**
   * @fn getGyroData
   * @brief get the gyro data 
   * @param data pointer to store the gyro data
   * @param timestamp pointer to store the timestamp for gryo
   * @return BMI160_OK(0) means succse
   */
  int8_t getGyroData(int16_t* data, uint32_t* timestamp);
  
  /**
   * @fn getAccelGyroData
   * @brief get the accel and gyro data 
   * @param data pointer to store the accel and gyro data
   * @return BMI160_OK(0) means succse
   */
  int8_t getAccelGyroData(int16_t* data);

  /**
   * @fn getAccelGyroData
   * @brief get the accel and gyro data 
   * @param data pointer to store the accel and gyro data
   * @param timestamp pointer to store the timestamp for accel and gyro
   * @return BMI160_OK(0) means succse
   */
  int8_t getAccelGyroData(int16_t* data, uint32_t* timestamp);

  /**
   * @fn softReset
   * @brief reset bmi160 hardware
   * @return BMI160_OK(0) means success
   */
  int8_t softReset();
  
  /**
   * @fn setInt
   * @brief set interrupt number and choosing step detector interrupt
   * @param intNum choose int1 or int2
   * @return BMI160_OK(0) means succse
   */
  int8_t setInt(int intNum);
  
  /**
   * @fn setStepCounter
   * @brief enable the step counter
   * @return BMI160_OK(0) means succse
   */
  int8_t setStepCounter();
  
  /**
   * @fn readStepCounter
   * @brief read the step counter from bmi160
   * @param stepVal pointer to store the step 
   * @return BMI160_OK(0) measn succse
   */
  int8_t readStepCounter(uint16_t *stepVal);
  /**
   * @fn setStepPowerMode
   * @brief set the step power model
   * @param model type of model 
   * @return BMI160_OK(0) measn succse
   */
  int8_t setStepPowerMode(uint8_t model);

  /* [MV] Moved to public */
  void   defaultParamSettg(struct bmi160Dev *dev);
  int8_t setAccelConf(struct bmi160Dev *dev);
  int8_t setGyroConf(struct bmi160Dev *dev);

  uint8_t onlyAccel=1;
  uint8_t onlyGyro=2;
  uint8_t bothAccelGyro=3;
  uint8_t stepNormalPowerMode=0;
  uint8_t stepLowPowerMode=1;

  private:
    int8_t I2cInit(struct bmi160Dev *dev);
    int8_t SPIInit();
    int8_t SPIInit(struct bmi160Dev *dev);
    
    int8_t softReset(struct bmi160Dev *dev);
    
    int8_t setSensConf();
    int8_t setSensConf(struct bmi160Dev *dev);
    
    int8_t checkAccelConfig(uint8_t *data, struct bmi160Dev *dev);
    int8_t processAccelOdr(uint8_t *data, struct bmi160Dev *dev);
    int8_t processAccelBw(uint8_t *data, struct bmi160Dev *dev);
    int8_t processAccelRange(uint8_t *data, struct bmi160Dev *dev);

      int8_t checkGyroConfig(uint8_t *data, struct bmi160Dev *dev);
    int8_t processGyroOdr(uint8_t *data, struct bmi160Dev *dev);
    int8_t processGyroBw(uint8_t *data, struct bmi160Dev *dev);
    int8_t processGyroRange(uint8_t *data, struct bmi160Dev *dev);

    int8_t setPowerMode(struct bmi160Dev *dev);
    int8_t setAccelPwr(struct bmi160Dev *dev);
    int8_t processUnderSampling(uint8_t *data, struct bmi160Dev *dev);
    int8_t setGyroPwr(struct bmi160Dev *dev);

    int8_t checkInvalidSettg( struct bmi160Dev *dev);

    int8_t getSensorData(uint8_t select_sensor, struct bmi160SensorData *accel, struct bmi160SensorData *gyro,struct bmi160Dev *dev);
    int8_t getAccelData(uint8_t len, struct bmi160SensorData *accel, struct bmi160Dev *dev);
    int8_t getGyroData(uint8_t len, struct bmi160SensorData *gyro, struct bmi160Dev *dev);
    int8_t getAccelGyroData(uint8_t len, struct bmi160SensorData *accel, struct bmi160SensorData *gyro, struct bmi160Dev *dev);

    int8_t getRegs(uint8_t reg_addr, uint8_t * data, uint16_t len, struct bmi160Dev *dev);
    int8_t setRegs(uint8_t reg_addr, uint8_t * data, uint16_t len, struct bmi160Dev *dev);

    int8_t I2cGetRegs(struct bmi160Dev *dev, uint8_t reg_addr, uint8_t *data, uint16_t len);
    int8_t I2cSetRegs(struct bmi160Dev *dev, uint8_t reg_addr, uint8_t *data, uint16_t len);

    int8_t SPIGetRegs(struct bmi160Dev *dev, uint8_t reg_addr, uint8_t *data, uint16_t len);
    int8_t SPISetRegs(struct bmi160Dev *dev, uint8_t reg_addr, uint8_t *data, uint16_t len);

    
    int8_t setInt(struct bmi160Dev *dev, int intNum);
    int8_t setIntConfig(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev);
    int8_t setAccelStepDetectInt(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev);
    int8_t enableStepDetectInt(struct bmi160AccStepDetectIntCfg *stepDetectIntCfg, struct bmi160Dev *dev);
    int8_t setIntrPinConfig(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev);
    int8_t configIntOutCtrl(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev);
    int8_t configIntLatch(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev);
    int8_t mapFeatureInterrupt(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev);
    int8_t configStepDetect(struct bmi160AccStepDetectIntCfg *stepDetectIntCfg, struct bmi160Dev *dev);

    int8_t setStepCounter(uint8_t step_cnt_enable, struct bmi160Dev *dev);
    int8_t setStepPowerMode(uint8_t model,struct bmi160Dev *dev);
    int8_t readStepCounter(uint16_t *stepVal, struct bmi160Dev *dev);
    

    struct bmi160Dev* Obmi160;
    struct bmi160SensorData* Oaccel;
    struct bmi160SensorData* Ogyro; 
};
