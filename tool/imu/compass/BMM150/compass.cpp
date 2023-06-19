/**
    This example
*/

#include "bmm150.h"
#include "bmm150_defs.h"

BMM150 bmm = BMM150();
bmm150_mag_data value_offset;

void setup() {
    if (bmm.initialize((char*)"/dev/i2c-1") == BMM150_E_ID_NOT_CONFORM) {
        printf("Chip ID can not read!\n");
	exit(-1);
    } else {
        printf("Initialize done!\n");
    }

    // Get those values by calling calibration
    value_offset.x = -31;
    value_offset.y = 6;
    value_offset.z = -37;

}

void loop() {
    bmm150_mag_data value;
    
    bmm.read_mag_data();

    value.x = bmm.raw_mag_data.raw_datax - value_offset.x;
    value.y = bmm.raw_mag_data.raw_datay - value_offset.y;
    value.z = bmm.raw_mag_data.raw_dataz - value_offset.z;

    float xyHeading = atan2(value.x, value.y);
    float zxHeading = atan2(value.z, value.x);
    float heading = xyHeading;

    if (heading < 0) {
        heading += 2 * PI;
    }
    if (heading > 2 * PI) {
        heading -= 2 * PI;
    }
    
    //float headingDegrees = heading * 180 / M_PI;
    //float xyHeadingDegrees = xyHeading * 180 / M_PI;
    //float zxHeadingDegrees = zxHeading * 180 / M_PI;
    //printf("Heading: %f; xyHeading: %f; zxHeading: %f\n", headingDegrees, xyHeadingDegrees, zxHeadingDegrees);

    printf("heading: %f\n", heading);
    fflush(stdout);
}


int main() {
  
  setup();
  usleep(100000);

  for(;;) {
    loop();
    usleep(100000);
  }
    return 0;
}

