#include <stdio.h>

#include "follow_line.h"

int line_sensor_threshold = 500;

// there are 5 line sensors (0-4) aligned in a row. the sensors are numbered from left to right. there are only straight lines. the three middle sensors should always be on the line. if the left or right sensor (sensor 1 or 3) is not on the line, the robot should turn left or right respectively. if the outer sensors detect a line, the robot has reached a waypoint.
void on_follow_line() {
    int keep_following = 1;

    while (keep_following) {
        // Read sensor values
        int val0 = i2c_linesensor_read("sensor0");
        int val1 = i2c_linesensor_read("sensor1");
        int val2 = i2c_linesensor_read("sensor2");
        int val3 = i2c_linesensor_read("sensor3");
        int val4 = i2c_linesensor_read("sensor4");
        
        // Check if the outer sensors detect a line (waypoint reached)
        if (val0 > line_sensor_threshold && val4 > line_sensor_threshold) {
            keep_following = 0;
            continue;
        }

        // Check if the robot needs to turn
        if (val1 < line_sensor_threshold) {
            robot_turn_left();
        } else if (val3 < line_sensor_threshold) {
            robot_turn_right();
        } else {
            robot_move_forward();
        }

        // Add a delay of 100ms
        usleep(100000);

        //TODO: handle obstacle or cone
    }
    uart_send("on_waypoint");
}

void robot_move_forward() {
    motor_left(50);
    motor_right(50);
}

void robot_turn_left() {
    motor_left(30);
    motor_right(60);
}

void robot_turn_right() {
    motor_left(60);
    motor_right(30);
}