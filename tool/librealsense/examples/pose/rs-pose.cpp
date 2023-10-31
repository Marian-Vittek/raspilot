/*
 if you see "undefined reference to `__atomic_load_8'" then link with:

cd ~/dev/raspilot/tool/librealsense/build/examples/pose ; make ; /usr/bin/c++  -pedantic -g -Wno-missing-field-initializers -Wno-switch -Wno-multichar -Wsequence-point -Wformat -Wformat-security -mfpu=neon -mfloat-abi=hard -ftree-vectorize -latomic -pthread -O3 -DNDEBUG -rdynamic CMakeFiles/rs-pose.dir/rs-pose.cpp.o -o rs-pose  -Wl,-rpath,/home/vittek/dev/raspilot/tool/librealsense/build: ../../librealsense2.so.2.50.0 -latomic

*/

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include "example-utils.hpp"

#include "string.h"
#include "linmath2.h"

// rotate around an axis -->  R = [sin(alpha/2)*x, sin(alpha/2)*y, sin(alpha/2)*z, cos(alpha/2)]
quat rotation = {0, -0.70710678118655, 0, 0.70710678118655};

// Translate pose and orientation to raspilot coordinate system and print it.
// The reported position/orientation must be in CS_GBASE coordinate system.
// The reported orientation is directly the orientation of the drone, raspilot is not doing any additional
// transformation on it, you have to take into  account the orientation how the sensor is mounted on the drone.
// The position is the position of the sensor in the same coordinate system as the orientation.

static void wikiQuaternionToEulerAngles(quat q, double *yaw, double *pitch, double *roll) {
    double x, y, z, w;
    double sinr_cosp, cosr_cosp, sinp, siny_cosp, cosy_cosp;
    
    x = q[0];
    y = q[1];
    z = q[2];
    w = q[3];
    
    // roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    *roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    // [MV] I had to change the sign here to get my pitch
    sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1) {
        *pitch = - copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    } else {
        *pitch = - asin(sinp);
    }
	
    // yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    *yaw = atan2(siny_cosp, cosy_cosp);
	
}


// Teh translation used here is for T265 pointing downward and usb connector is on the right side of the drone.
// I determined the translation by experimenting, I have no idea why the translation is like that.
void translateAndPrintPose(double x, double y, double z) {
  vec3 r, p;
  p[0] = -z;
  p[1] = -x;
  p[2] = y;
  printf("pose %f %f %f\n", p[0], p[1], p[2]);
}
void translateAndPrintOrientation(double x, double y, double z, double w) {
  quat p, q, i;
  double roll, pitch, yaw;
  q[0] = -z;
  q[1] = -x;
  q[2] = y;
  q[3] = w;
  quat_mul(p, q, rotation);
  // printf("quat %f %f %f %f\n", p[0], p[1], p[2], p[3]);
  wikiQuaternionToEulerAngles(p, &yaw, &pitch, &roll);
  printf("rpy %f %f %f\n", roll, pitch, yaw);
}

int main(int argc, char * argv[]) try
{
    std::string serial;
    if (!device_with_streams({ RS2_STREAM_POSE}, serial))
        return EXIT_SUCCESS;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);

    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // Print the x, y, z values of the translation, relative to initial position
        //std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
        //    pose_data.translation.y << " " << pose_data.translation.z << " (meters)";
	translateAndPrintPose(pose_data.translation.x, pose_data.translation.y, pose_data.translation.z);
	translateAndPrintOrientation(pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z, pose_data.rotation.w);
	fflush(stdout);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
