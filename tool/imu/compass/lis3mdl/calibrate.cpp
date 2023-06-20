#include "LIS3MDL.h"

#define delay(x) usleep(1000*(x))
#define min(x,y) ((x)<(y)?(x):(y))
#define max(x,y) ((x)>(y)?(x):(y))

LIS3MDL mag;
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

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

  running_min.x = min(running_min.x, mag.m.x);
  running_min.y = min(running_min.y, mag.m.y);
  running_min.z = min(running_min.z, mag.m.z);

  running_max.x = max(running_max.x, mag.m.x);
  running_max.y = max(running_max.y, mag.m.y);
  running_max.z = max(running_max.z, mag.m.z);

  printf("min: {%+6d, %+6d, %+6d}   max: {%+6d, %+6d, %+6d}\n",
	 running_min.x, running_min.y, running_min.z,
	 running_max.x, running_max.y, running_max.z
	 );
  delay(100);
}



int main() {

  setup();
  usleep(100000);
  for(;;) {
    loop();
  }
}

