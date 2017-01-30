# Warmup Project 2017
Riley Chapman and Bonnie Ishiguro

Computational Robotics, Olin College of Engineering 

## Teleop
Goal: Control the linear and angular velocity of the robot using keyboard commands

The script interprets keyboard input, translates these commands to linear and angular velocity commands, and publishes a `Twist` message to the `/cmd_vel` ROS topic.

Since this script is not structured in an object-oriented way, it utilizes a few global variables to maintain consistency in values such as the maximum linear and angular velocity. 

## Driving in a Square
Goal: Drive the robot in a square, navigating based on location in the `odom` coordinate frame. 

At this point in the class, the `odom` coordinate frame is as close as we have to a "world" coordinate frame. We programed the robot to have two modes. The first mode controls the angle of the robot, used for turning in place at the corners of the square. The second mode drives the robot toward a target point. This decision to control the robot in the `odom` coordinate frame enables the robot to be robust to disturbances while driving in the square, as long as the disturbances are detected in the `odom` frame (i.e. overriding via tele-op).

The "turn in place" mode uses proportional control on the angular velocity by calculating the error in orientation between the angle the robot is facing, and some goal angle. This control scheme enables the robot to turn to any orientation in the `odom` coordinate frame. 

The "drive to point" mode utilized proportional control on both the angular and linear velocities. Angular velocity is controlled by the error in orientation found by taking the difference between the orientation of the robot and the direction the point is from the robot. Linear velocity (in the x direction of the robot's frame of reference, `base_link`) is controlled by the distance between the goal point and the robot. 

The robot controller switches between these two modes to alternate between turning in the direction of the next side, and driving along that side to the next corner. 


## Wall Following
Goal: Drive the robot parallel to any wall detected by its LIDAR scanner

To detect an existing wall, we read from two separate directions from the LIDAR scanner, 45 degrees apart.  We calculate the angle of the wall relative to the robot using the distances between these detected wall points and the robot.  The angular velocity is proportionally controlled by the difference between the orientation of the robot and the orientation of the wall.  This results in the robot orienting itself parallel to the wall.  We set a constant linear velocity so that it the robot will drive forward while orienting itself in this fashion.  To handle dropped scanner data, we only update the distance values from the LIDAR scanner when they are non-zero.

The major challenges of this part of the project included determining how to calculate the relative angle of the wall based on the distances returned from the LIDAR scanner.

## Person Following
coming soon

## Obstacle Avoidance
coming soon

## Finite State Control
coming soon
