/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low magnetometer data registers.
They can be converted to units of gauss using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An LIS3MDL gives a magnetometer X axis reading of 1292 with its
default full scale setting of +/- 4 gauss. The GN specification
in the LIS3MDL datasheet (page 8) states a conversion factor of 6842
LSB/gauss (where LSB means least significant bit) at this FS setting, so the raw
reading of 1292 corresponds to 1292 / 6842 = 0.1888 gauss.
*/

#include "LIS3MDL.h"

#define delay(x) usleep(1000*(x))

LIS3MDL mag;

void setup()
{
  int r;

  r = mag.init((char*)"/dev/i2c-1", LIS3MDL::device_LIS3MDL, LIS3MDL::sa1_high);
  if (r == 0) {
    printf("Failed to detect and initialize magnetometer!\n");
    exit(-1);
  }

  mag.enableDefault();
}

void loop()
{
  mag.read();

  printf("M: %6d %6d %6d\n", mag.m.x, mag.m.y, mag.m.z);
  fflush(stdout);
  delay(100);
}

int main() {

  setup();
  usleep(100000);
  for(;;) {
    loop();
  }
}

