# course-code-ASS_Lokalisasi-dan-pemetaan
#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/CircleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/factory/containers/protos/WoodenBox.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"

WorldInfo {
  basicTimeStep 64
}
Viewpoint {
  orientation 0.18519842932628786 0.4848315204687159 -0.8547747881957382 0.8405191784381161
  position -2.8544834673716877 3.0985603179348127 1.7376540045736695
  follow "TurtleBot3Burger"
  followType "Pan and Tilt Shot"
}
TexturedBackground {
}
TexturedBackgroundLight {
}
CircleArena {
  radius 3
  wallHeight 0.4
}
WoodenBox {
  translation 0.467367 -0.545426 0.15
  size 0.3 0.3 0.3
}
WoodenBox {
  translation 1.26618 1.07342 0.15
  name "wooden box(1)"
  size 0.3 0.3 0.3
}
WoodenBox {
  translation -0.15697 0.782967 0.15
  name "wooden box(2)"
  size 0.3 0.3 0.3
}
WoodenBox {
  translation -1.62271 1.08968 0.15
  name "wooden box(3)"
  size 0.3 0.3 0.3
}
WoodenBox {
  translation -1.09887 -0.301011 0.15
  name "wooden box(4)"
  size 0.3 0.3 0.3
}
WoodenBox {
  translation -2.71307 -0.22263 0.15
  name "wooden box(7)"
  size 0.3 0.3 0.3
}
WoodenBox {
  translation -0.744103 2.69736 0.15
  name "wooden box(8)"
  size 0.3 0.3 0.3
}
WoodenBox {
  translation 2.30103 0.203241 0.15
  name "wooden box(9)"
  size 0.3 0.3 0.3
}
WoodenBox {
  translation 0.005323 -1.57959 0.15
  name "wooden box(5)"
  size 0.3 0.3 0.3
}
WoodenBox {
  translation -1.0974 -1.8598 0.15
  name "wooden box(6)"
  size 0.3 0.3 0.3
}
TurtleBot3Burger {
  hidden position_0_0 152.38815535273102
  hidden position_0_1 181.8030507780411
  hidden position_0_2 -656.5187129551695
  hidden position2_0_2 -0.9361372486467726
  hidden position3_0_2 -357.3911195541619
  hidden linearVelocity_0 0.03357487620184966 0.03642537311949538 -2.7132208027658466e-08
  hidden angularVelocity_0 -4.053808252186808e-07 -1.6359192070790737e-06 -0.0488736918847002
  hidden position_1_0 3258.475520000073
  hidden position_1_1 6343.045120000139
  hidden rotation_2 0.22957447555416133 0.6882194710131156 0.6882219989890318 3.5929194620408973
  hidden translation_3 -0.09134037811487736 -0.01 0.007762094588443925
  hidden rotation_3 -0.05998176784485747 -0.7058323198666258 -0.705834912538267 3.0217731159668033
  hidden rotation_8 0 0.9999999999999999 0 0.020911980420939627
  hidden linearVelocity_8 0.03198215040424019 0.03249285485053435 1.1449990970568233e-07
  hidden angularVelocity_8 -0.9753482583644207 0.9602338103791497 -0.04989484932978622
  hidden rotation_9 0.5486375603084271 4.728880217804885e-07 0.8360603013052308 3.1415972100583422
  hidden linearVelocity_9 0.037473424091401634 0.038061127474126856 -1.1287311403967216e-07
  hidden angularVelocity_9 -1.1626144929563629 1.1443141602944291 -0.05026050555468928
  hidden rotation_10 0.8435683629642515 -0.3141280991303208 -0.43556394977383533 2.8718773754918683
  hidden linearVelocity_10 0.031918564946732154 0.038047526487083906 -6.615907458774152e-08
  hidden angularVelocity_10 -9.511881623716928 7.979641234612101 0.0005823605154012243
  translation -1.3021148641155202 0.8393003503766984 -0.0004935440174139924
  rotation 0.004650236259743506 -0.01121909125901626 0.9999262509275623 0.7915949046889869
  controller "turtlebot3_ostacle_avoidance"
}
/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define BASE_SPEED 1.5

// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

int main(int argc, char **argv) {
  wb_robot_init();

  // get and enable the lidar
  WbDeviceTag lidar = wb_robot_get_device("LDS-01");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);

  // get lidar motor and enable rotation (only for visualization, no effect on the sensor)
  WbDeviceTag lidar_main_motor = wb_robot_get_device("LDS-01_main_motor");
  WbDeviceTag lidar_secondary_motor = wb_robot_get_device("LDS-01_secondary_motor");
  wb_motor_set_position(lidar_main_motor, INFINITY);
  wb_motor_set_position(lidar_secondary_motor, INFINITY);
  wb_motor_set_velocity(lidar_main_motor, 30.0);
  wb_motor_set_velocity(lidar_secondary_motor, 60.0);

  // get the motors and enable velocity control
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_motor_set_velocity(left_motor, 0.0);

  const int lidar_width = wb_lidar_get_horizontal_resolution(lidar);
  const double lidar_max_range = wb_lidar_get_max_range(lidar);

  // init braitenberg coefficient
  double *braitenberg_coefficients = (double *)malloc(sizeof(double) * lidar_width);
  int i;
  for (i = 0; i < lidar_width; i++)
    braitenberg_coefficients[i] = 6 * gaussian(i, lidar_width / 4, lidar_width / 12);

  while (wb_robot_step(TIME_STEP) != -1) {
    double left_speed = BASE_SPEED, right_speed = BASE_SPEED;

    // get lidar values
    const float *lidar_values = wb_lidar_get_range_image(lidar);

    // apply the braitenberg coefficients on the resulted values of the lidar
    for (i = 0.25 * lidar_width; i < 0.5 * lidar_width; i++) {
      const int j = lidar_width - i - 1;
      const int k = i - 0.25 * lidar_width;
      if (lidar_values[i] != INFINITY && !isnan(lidar_values[i]) && lidar_values[j] != INFINITY && !isnan(lidar_values[j])) {
        left_speed +=
          braitenberg_coefficients[k] * ((1.0 - lidar_values[i] / lidar_max_range) - (1.0 - lidar_values[j] / lidar_max_range));
        right_speed +=
          braitenberg_coefficients[k] * ((1.0 - lidar_values[j] / lidar_max_range) - (1.0 - lidar_values[i] / lidar_max_range));
      }
    }

    // apply computed velocities
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  };

  free(braitenberg_coefficients);
  wb_robot_cleanup();

  return 0;
}
