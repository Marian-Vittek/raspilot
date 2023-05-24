// libraries
#include "bmm150.h"
#include "bmm150_defs.h"

BMM150 bmm = BMM150();
bmm150_mag_data value_offset;

/**
    @brief Do figure-8 calibration for a limited time to get offset values of x/y/z axis.
    @param timeout - seconds of calibration period.
*/
void calibrate(int n) {
  int i;
    int16_t value_x_min = 0;
    int16_t value_x_max = 0;
    int16_t value_y_min = 0;
    int16_t value_y_max = 0;
    int16_t value_z_min = 0;
    int16_t value_z_max = 0;
    uint32_t timeStart = 0;

    bmm.read_mag_data();
    value_x_min = bmm.raw_mag_data.raw_datax;
    value_x_max = bmm.raw_mag_data.raw_datax;
    value_y_min = bmm.raw_mag_data.raw_datay;
    value_y_max = bmm.raw_mag_data.raw_datay;
    value_z_min = bmm.raw_mag_data.raw_dataz;
    value_z_max = bmm.raw_mag_data.raw_dataz;
    usleep(1000*100);

    for(i=0; i<n; i++) {
        bmm.read_mag_data();

        /* Update x-Axis max/min value */
        if (value_x_min > bmm.raw_mag_data.raw_datax) {
            value_x_min = bmm.raw_mag_data.raw_datax;
            // printf("Update value_x_min: ");
            // printfln(value_x_min);

        } else if (value_x_max < bmm.raw_mag_data.raw_datax) {
            value_x_max = bmm.raw_mag_data.raw_datax;
            // printf("update value_x_max: ");
            // printfln(value_x_max);
        }

        /* Update y-Axis max/min value */
        if (value_y_min > bmm.raw_mag_data.raw_datay) {
            value_y_min = bmm.raw_mag_data.raw_datay;
            // printf("Update value_y_min: ");
            // printfln(value_y_min);

        } else if (value_y_max < bmm.raw_mag_data.raw_datay) {
            value_y_max = bmm.raw_mag_data.raw_datay;
            // printf("update value_y_max: ");
            // printfln(value_y_max);
        }

        /* Update z-Axis max/min value */
        if (value_z_min > bmm.raw_mag_data.raw_dataz) {
            value_z_min = bmm.raw_mag_data.raw_dataz;
            // printf("Update value_z_min: ");
            // printfln(value_z_min);

        } else if (value_z_max < bmm.raw_mag_data.raw_dataz) {
            value_z_max = bmm.raw_mag_data.raw_dataz;
            // printf("update value_z_max: ");
            // printfln(value_z_max);
        }

        printf(".");fflush(stdout);
        usleep(1000*100);

    }

    value_offset.x = value_x_min + (value_x_max - value_x_min) / 2;
    value_offset.y = value_y_min + (value_y_max - value_y_min) / 2;
    value_offset.z = value_z_min + (value_z_max - value_z_min) / 2;

}

void setup() {

    if (bmm.initialize((char*)"/dev/i2c-1") == BMM150_E_ID_NOT_CONFORM) {
        printf("Chip ID can not read!\n");
        exit(-1);
    } else {
        printf("Initialize done!\n");
    }

    printf("Start figure-8 calibration after 3 seconds.\n");
    usleep(1000*3000);
    calibrate(300);
    printf("\nCalibrate done..\n");

    printf("Copy paste the following ito compass.cpp\n");
    printf("\n");
    printf("value_offset.x = %d;\n", value_offset.x);
    printf("value_offset.y = %d;\n", value_offset.y);
    printf("value_offset.z = %d;\n", value_offset.z);
    printf("\n");
    
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
    float headingDegrees = heading * 180 / M_PI;
    float xyHeadingDegrees = xyHeading * 180 / M_PI;
    float zxHeadingDegrees = zxHeading * 180 / M_PI;

    printf("Heading: %f\n", headingDegrees);

    usleep(1000*100);
}


int main() {
  setup();
  
  usleep(100000);

  for(;;) {
    loop();
  }
}

