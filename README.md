# Computational Robotics Warmup Project Writeup

Kevin Zhang and Shane Kelly

02/06/17

## Robot Teleoperation

**The problem:**

When not running other programs, the Neato should be able to be repositioned from user key input.

**Our solution:**

In the style of a finite state machine, depending on the input, a certain output will occur. In this case, the input is the user pressing certain keys and the output is the linear and angular motion of the Neato. We designated the input to output mappings with a series of if statements.

**Code architecture:**

For teleoperation, our code uses OOP structure with a main Teleop class with an init method that initializes member variables, a getKey method that reads in user input, and an act method that maps the user-given key to a state of robot motion. In our code’s main we have a while loop that runs the getKey and act methods on repeat.

**Challenges:**

It is difficult to make clean-looking teleoperation code that has many possible inputs (key presses from the user) and has many possible outputs (robot motions). A behvior that needs to check for so many possible states often becomes cluttered with unintuitive if-else statements, which is a problem that we encountered when completeing this task.

**Possible improvement:**

One improvement to this teleoperation code would be to make the key input only last for as long as the key is pressed. Currently, when you let go of the forward button, the robot still goes forward until the next command is given. We could change that such that when you let go of the forward button, the robot stops moving.

## Driving in a Square

**The problem:**

The Neato should autonomously trace the perimeter of a 1m x 1m square.

**Our solution:**

One of the largest considerations when solving this problem is determining how to tell when the Neato has turned far enough for a 90° angle and when the Neato has moved far enough for a 1m side. If we assumed the Neato was moving at a constant velocity (which we managed by operating the Neato at a very low speed), then we could control precise angular and linear displacement by controlling how long the Neato moves. We decided that using odometry data from the encoders in the Neato’s wheels would be more difficult to implement and not much more effective over the short time frame of making a single square. Our code architecture resembled an FSM where the input was the current state of the robot, the current time, and the time since an action was last taken. Depending on these inputs the robot either moved forward or turned.

![drive_square](images/drive_square.jpg?raw=true "Drive Square")

**Code architecture:**

For driving in a square, our code uses OOP structure with a main Square class with an init method that initializes member variables (state variable and speed variables), and an act method that looks at the current state of the robot and the time since an action was last taken in order to push the robot into a new state. In our code’s main we have a while loop that runs the act method on repeat.

**Challenges:**

The most difficult part of this behavior is getting the robot to acurately perfom a 90 degree turn and accurately move 1 meter forward. In order to combat this challenge with our time-based movement, we had the robot move slowly in hopes that there would be minimal drift in the wheels when we gave the robot start and stop commands.

**Possible improvement:**

The biggest improvement that we could make to the drive square behavior would be to use odometry information to perform movements instead of time-based motion. This would make our square more accurate and more repeatable for many repetitions.

**Key takeaways:**

The key takeaway for this task is that time-based procedures are often very error-prone. Using a method of actuation that allows errors to stack is often doomed to fail.

## Wall Following

**The problem:**

When starting from a position near a wall, the Neato should be able to navigate along the wall at a set distance.

**Our solution:**

We split this problem into two separate challenges: orient parallel to the wall and move to a target distance away from the wall. The Neato is parallel to the wall if two lidar readings, of equal angle relative to the Neato’s 270° mark, are equal in magnitude. We chose the readings at 240° and 300°, and used proportional control to attempt to make the difference between those two readings equal zero. We used a similar proportional controller to optimize a distance reading to the wall minus our set target distance to zero. We kept a live visualization of where the robot sensed the wall was at all times in order to more easily debug the behavior of our code. The position of the wall was calculated by drawing a line between our 240° and 300° measurements.

![wall_following](images/wall_following.jpg?raw=true "Wall Follow")

**Code architecture:**

Our code uses OOP structure with a main Wall_Follower class with an init method that initializes member variables (p-controller variables and lidar readings), a processScans method that acts as the lidar subscriber callback, and an act method that applies a p-controller to the angular velocity of the robot based on our lidar readings and publishes linear and angular velocities to the Neato. In our code’s main we have a while loop that runs the act method on repeat.

**Challenges:**

Our wall following relies solely on lidar input, but there are often obstacles along the wall that do not scan very well with lidar, such as some black materials of various textures. This makes calculating an accurate wall position very difficult. Another challenge that we dealth with was creating a proportional controller that received two inputs (angle to the wall and distance to the wall) and tuned one output (angular velocity). We eventually created an equation to appropriately balance the two inputs to put into our p-controller.

**Possible improvement:**

While our wall following program works incredibly well, we could always add additional functionality to it. One possiblity is to give the wall follower a time-dependent distance to follow the wall at as opposed to a static distance. The robot could be given a sine function and it would trace the curve as if the wall were the x-axis on a graph.

**Key takeaways:**

A key takeaway from this task is that using the correct approach can make a very difficult problem almost trivial. Our first attempt at wall following was simply p-controlling a single reading 90 degrees to the right of the robot. This was ineffective and the reading changed unpredictably based on our angular velocity. When we redefined our approach to include comparing two lidar scans to find a parallel path along the wall, then the problem became almost effortlessly solved.

## Person Following

**The problem:**

For this behavior, the Neato must be able to distinguish a human apart from walls and other objects, and then follow the detected human even when new objects enter its field of vision.

**Our solution:**

Our strategy was to have the Neato “calibrate” on a person’s feet, creating a centroid that it follows around. Then when the person moves around, their feet scans will move. The tricky decision was figuring out how to maintain the object of interest on the feet once the feet moved. So we came up with a closest centroid solution. The Neato will collect a round of scans and then filter out all scans except the eight closest points to the centroid of the previous scan. In this way, we make sure that only the person’s feet will remain in subsequent scans, so even if new objects appear, as long as the person doesn’t move too drastically, their feet will still be the closest points to the previous centroid, allowing the Neato to distinguish between human and other objects.

![person_following](images/person_following.jpg?raw=true "Person Follow")

**Code architecture:**

Our Code used a PersonFollower object which held the ROS node, the publishers, and the subscribers, as well as the act method. Upon initialization, the PersonFollower object starts the node, publishers, and subscribers. The act() method of the object then runs in a loop, and its data that’s sent through the publishers will change depending on the subscriber callbacks.

**Challenges:**

Our greatest challenge was figuring out how to distinguish feet from another object. Since we’re only using the Laser Scan as our primary sensor, we had to come up with a clever way to make it figure out that a specific cluster of points was the feet, and then in the next set of scans, when the feet have moved and the data points are different, that the Neato still knows that the moved cluster is still the feet. We solved this using the closest centroid solution described above.

**Possible improvement:**

Our biggest flaw in our code was that the person must move slowly for the robot to follow them. Since we use the closest points, if the person moves too drastically such that the person’s feet were no longer the closest points to the previous centroid, and say, a wall was nearby, it’s highly possible that the Neato will begin to follow the wall. And unfortunately, it seems that our parameters made it such that rate at which a person could walk for the Neato to continue following him was slower than a normal walking pace. Steps for improvement include doing some more fine-tuning on the parameters in our code, which could potentially make the Neato move faster and be more responsive to changes in the centroid such that the difference between the previous feet scans and the next were still in a valid range. We could also look at adding in the camera and using that as a secondary sensor to check the position of the feet, or we could explore different ways of using the Laser Scans, such as projecting the scans into the Odom frame and then looking for motion that is unique to the feet.

**Key takeaways:**

The key takeaway is that one sensor is relatively limiting to the pursuit of a goal for a project. In this case, having a camera would’ve made things more robust. Potentially looking into different ways of sensor fusion, and also understanding the limitations of the code are good things to consider for next time.

## Obstacle Avoidance

**The problem:**

For this behavior, the Neato must be able to avoid any obstacle it encounters in front of it, such that it will never hit an object, potentially with the goal of reaching a pre-determined point.

**Our solution:**
Our strategy included a large pivot. Our first approach was to try for the stretch goal and make the Neato move towards a point while avoiding any obstacles along the way. We explored the concept of potential fields and came up with the correct approach. Basically find a goal point in the Odom frame, and convert that into the Base-Link frame. Then point a vector from the Neato to the goal point, which is the preferred motion vector. Then have all nearby objects become repulsive forces on the Neato which create a net force vector the Neato chooses to move in the direction and magnitude of. We managed to solve the repulsive forces and net forces portions, but were stuck on the creating the goal point vector in the Base-Link frame. In the interest of time, we pivoted to the simpler approach; the Neato just moves forward, and when it encounters an obstacle, it will either move left or right until it has side-stepped the object, and then it will continue to move forward in the original direction again.

![obstacle_avoidance](images/obstacle_avoidance.jpg?raw=true "Obstacle Avoidance")

**Code architecture:**

Our Code used a ObstacleAvoider object which held the ROS node, the publishers, and the subscribers, as well as the act method. Upon initialization, the ObstacleAvoider object starts the node, publishers, and subscribers. The act() method of the object then runs in a loop, and its data that’s sent through the publishers will change depending on the subscriber callbacks.

**Challenges:**

Our  greatest challenge was mainly finding the goal point vector in the stretch goal. We managed to derive the correct approach, but by the time we did, it had already been 3 hours, and we realized that we would need another 3-5 hours before this one module was finished. So we decided to switch to the simpler approach to save time. The simpler approach didn’t have many problems, but it would’ve been cooler had we more time to work on the more robust method.

**Possible improvement:**

The simpler approach’s greatest flaw is that it really only works when there are things right in front of it that it must sidestep. The code logic is a bit more hardcoded, so it only avoids obstacles under certain conditions. For example, if the Neato encountered a corner of a wall, there’s a high probability that it just turn into the corner. In addition, the simpler approach just has the Neato blindly moving forward without any sort of goal, makes it less complete in a sense.
The most logical step to improve would just be to implement our derivation of the potential fields method. It would’ve been possible if we had a couple more days, and we checked with other teams and confirmed that our theoretical solution was correct. It was really just a matter of time.

**Key takeaways:**

The key takeaway is that it’s good to know your own limitations and your own goals towards a project. We pushed a little too hard in a direction that we weren’t really ready to tackle, so we ended up wasting a good chunk of time on a thing that was cool and we figured out but was never implemented. Learning proper scoping is a good idea.

## Finite State Control

**The problem:**

For finite state control, the idea is to have the Neato be able to switch between different states and perform multiple behaviors autonomously, changing between the behaviors based on external stimuli.

**Our solution:**

Our approach for our finite state machine was to combine driving a square and person following. The two states were drive_square, where the Neato would continuously driving a square formation, and person_follow, where the Neato would follow a person around. The stimulus to change between states was whether or not the Neato detected something in front of it within the field of vision of the person_follow module. It did, then it meant that was a person was in front, so it would begin to follow the person. Once the person jumped out of the way, the Neato would not longer detect anything, and thus would revert back to driving a square until another person stepped into its path. We decided to specifically use this pair of behaviors because it made logical sense out of the behaviors we had made previously. Wall following can’t continue indefinitely because at some point the Neato runs out of wall to follow, and our implementation of obstacle avoidance was done in a way that made it only work under specific conditions, so combining with another behavior was pragmatically unfeasible. Driving a square can continue as a passive state indefinitely, and the person follow state distinguishes itself  from the driving square state very clearly in that a person will enter the Neato’s field of vision. The tricky part was making sure that the Neato would not revert too early back to driving a square because the person got too far away from the Neato’s field of vision, and we had to make sure that while the Neato was driving a square it wouldn’t pick up any other objects as persons. We tuned parameters until we found a good balance between the two states.

![finite_state_controller](images/finite_state_controller.jpg?raw=true "Finite State Controller")

**Code architecture:**

Our Code used a FiniteStateMachine object which held the ROS node, the publishers, and the subscribers, as well as the act method. Upon initialization, the FiniteStateMachine object starts the node, publishers, and subscribers. The act() method of the object then runs in a loop, and its data that’s sent through the publishers will change depending on the subscriber callbacks. The callbacks and the act( ) method contain high-level conditionals that will implement one of the behaviors depending on the current state of the Neato. Another way we could’ve done this was to use ROS topics between our existing behavior scripts to communicate what state the Neato was in, but we deemed that just straight up combining the two scripts into one and then segregating them by conditionals was easier and had a higher chance of success.
 
**Challenges:**

Our greatest challenge was balancing the detection between person_follow and driving_square. Often times during testing the Neato would be driving a square and then just detect a table leg or a wall, which meant that we needed to reduce the range of detection. However, if we reduced it too much, then during person following, the person being followed could only move in small baby steps very slowly because the range of the Neato was too small to detect a more comfortable walking pace. There was a lot of fine tuning that went into figuring out how to balance these two issues and ensure smooth state transitions.

**Possible improvement:**

Our greatest flaw was that changing of states was based solely on whether something was detected in the Neato field of vision, which meant that anything that went into the range of person_follow detection would trigger a change of state. In that sense our detection for state change was a little tenuous and delicate. In addition, while logical for a FSM pair, the driving square module is a little hard coded, so sometimes when switching back from person following, the Neato will just driving a square into a wall because it lost sense of its orientation. Steps to improve include using a more robust behavior instead of driving square or making driving square a bit more smart such it knew it orientation and wouldn’t crash into stuff after leaving person_following. At the same time more work could be done to make the detection of feet more robust, such that the Neato can continue to drive squares and detect objects as long as the objects aren’t feet. This could be done using motion detection in the Odom frame, or using a camera and sensor fusion.

**Key takeaways:**

The key takeaway is to remember that the Neato is not smart unless the programmer made it so. There were definitely a couple of holes in this module that occurred because we weren’t thinking fully about what the Neato sees and what state transitions should look like. When creating more complex behavior patterns, recognizing how different behaviors interact with each other and taking that into account is a very useful tidbit.
