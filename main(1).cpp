#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/display.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/position_sensor.hpp>
#include <webots/Camera.hpp>



#define TIME_STEP 20
#define MAX_SPEED 6.28

using namespace webots;


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

   //wheels
  Motor *leftMotor = robot->getMotor("motor1");
  Motor *rightMotor = robot->getMotor("motor2");

  // set robots initial velocity
  leftMotor->setVelocity(1.0);
  rightMotor->setVelocity(1.0);

  // get robots initial position
    leftMotor = wb_robot_get_device("left wheel motor");
    rightMotor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

  // set robots initial velocity
    wb_motor_set_velocity(leftMotor, 1.0);
    wb_motor_set_velocity(rightMotor, 1.0);

  //goal position of the robot
   float goal_x = -1.2;
   float goal_y = -1.2;

 // robot get position sensors and enable them
 left_position_sensor = wb_robot_get_device("left wheel sensor");
 right_position_sensor = wb_robot_get_device("right wheel sensor");
 wb_position_sensor_enable(left_position_sensor, TIME_STEP);
 wb_position_sensor_enable(right_position_sensor, TIME_STEP);


   //initial  position if the robot
   ot.result.x = 1;
   ot.result.y = 1;
   ot.result.theta = 0;

   // get display  of robot
    display = wb_robot_get_device("display");
    init_display();
    printf("Reset OK\n");

  //Distance Sensor
  DistanceSensor *ds[2];
  char dsNames = {"dsL", "dsR"};
  for (int i = 0; i < 2; i ++) {
  ds[i] = robot->getDistanceSensor("dsL" , "dsR");
  ds[i] = enable(TIME_STEP);

  Camera *cm;
  cm = robot->getCamera("camera");
  cm -> enable(TIME_STEP);
  cm -> recognitionEnable(TIME_STEP);


 // defining the values of the attractive potential
 float d = 1
 float d_goal = sqrt((goal_x - pos_x) * (goal_x - pos_x) + (goal_y - pos_y) * (goal_y - pos_y));

 // robot control code with attarctive potential and obstacle avoidance using Camera

 while { (U_att = 0.5 (((int_x - goal_x) * 2) + ((int_y - goal_y) * 2)); //quadratic potential

 if (d_goal  < 1);
  U_grad = sqrt((goal_x-int_x)*2) + (goal_y-int_y)*2); // gradient of attractive potential

  else if {
  //obstacle avoidance
  int obstacleavoidanceCounter = 0;

  while (robot->step(TIME_STEP) != -1) {
   double leftspeed = 1.0;
   double  rightspeed = 1.0;
   if (obstacleavoidanceCounter > 0) {
   leftspeed = -1.0;
   rightspeed = 1.0;
   }
   else // follow sensor readings
   for (int i = 0; i < 2; i++) {
   if (ds [i]->getValue() <  1000.0)

    }

  };
}

else if (d_goal = 1){
  U_att = (sqrt((int_x - goal_x) * 2) +  ((int_y - goal_y) * 2)) - 0.5)
  U_grad = sqrt((goal_x-int_x)*2) + (goal_y-int_y)*2); // gradient of attractive potential

}
  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
