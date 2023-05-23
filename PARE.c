#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>
#include <webots/led.h>

#define TIME_STEP 256
#define MAX_SPEED 6.28
#define OBSTACLE_THRESHOLD 100
#define QtddLeds 10

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0],-1);

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  WbDeviceTag proximity_sensors[8];
  char proximity_sensor_names[8][5] = {
    "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"
  };

  WbNodeRef objectNode = wb_supervisor_node_get_from_def("wooden_box");

  //printf("Posição do objeto: x = %f, y = %f, z = %f\n", x, y, z);
  
  
  for (int i = 0; i < 8; ++i) {
    proximity_sensors[i] = wb_robot_get_device(proximity_sensor_names[i]);
    wb_distance_sensor_enable(proximity_sensors[i], TIME_STEP);
  }
  
  const double tolerance = 1e-4;
  const double *position = wb_supervisor_node_get_position(objectNode);
  double xb = position[0];
  double yb = position[1];
  double x = xb;
  double y = yb;
    
  
  while (wb_robot_step(TIME_STEP) != -1) {
    double sensor_values[8];
  
 
    const double *box = wb_supervisor_node_get_position(objectNode);
    xb = box[0];
    yb = box[1];
    
    for (int i = 0; i < 8; ++i) {
      sensor_values[i] = wb_distance_sensor_get_value(proximity_sensors[i]);
    }

    double left_speed = MAX_SPEED;
    double right_speed = MAX_SPEED;
 
    if (sensor_values[0] > OBSTACLE_THRESHOLD || sensor_values[7] > OBSTACLE_THRESHOLD) {

      left_speed *= -1.0;
      right_speed *= 0.5;
    } 
    else if (sensor_values[6] > OBSTACLE_THRESHOLD) {

      left_speed *= 0.5;
      right_speed *= -1.0;
    } 
    else if (sensor_values[1] > OBSTACLE_THRESHOLD) {
      left_speed *= -1.0;
      right_speed *= 0.5;
    }
    if(fabs(xb - x) > tolerance || fabs(yb - y) > tolerance)
    {
      left_speed *= 0;
      right_speed *= 0;
      wb_led_set(Leds[0],wb_led_get(Leds[0])*-1);
    }
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}