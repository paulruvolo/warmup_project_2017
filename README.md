# Computational Robotics Warmup Project Writeup

Kevin Zhang and Shane Kelly

02/06/17

## Robot Teleoperation
**The problem:**

When not running other programs, the Neato should be able to be repositioned from user key input.

**Our solution:**

In the style of a finite state machine, depending on the input, a certain output will occur. In this case, the input is the user pressing certain keys and the output is the linear and angular motion of the Neato. We designated the input to output mappings with a series of if statements.

## Driving in a Square

![drive_square](images/drive_square.jpg?raw=true "Drive Square")

**The problem:**

The Neato should autonomously trace the perimeter of a 1m x 1m square.

**Our solution:**

One of the largest considerations when solving this problem is determining how to tell when the Neato has turned far enough for a 90° angle and when the Neato has moved far enough for a 1m side. If we assumed the Neato was moving at a constant velocity (which we managed by operating the Neato at a very low speed), then we could control precise angular and linear displacement by controlling how long the Neato moves. We decided that using odometry data from the encoders in the Neato’s wheels would be more difficult to implement and not much more effective over the short time frame of making a single square. Our code architecture resembled an FSM where the input was the current state of the robot, the current time, and the time since an action was last taken. Depending on these inputs the robot either moved forward or turned.


## Wall Following

![wall_following](images/wall_following.jpg?raw=true "Wall Follow")

**The problem:**

When starting from a position near a wall, the Neato should be able to navigate along the wall at a set distance.

**Our solution:**

We split this problem into two separate challenges: orient parallel to the wall and move to a target distance away from the wall. The Neato is parallel to the wall if two lidar readings, of equal angle relative to the Neato’s 270° mark, are equal in magnitude. We chose the readings at 240° and 300°, and used proportional control to attempt to make the difference between those two readings equal zero. We used a similar proportional controller to optimize a distance reading to the wall minus our set target distance to zero. We kept a live visualization of where the robot sensed the wall was at all times in order to more easily debug the behavior of our code. The position of the wall was calculated by drawing a line between our 240° and 300° measurements.


## Person Following

![person_following](images/person_following.jpg?raw=true "Person Follow")

**The problem:**

For this behavior, the Neato must be able to distinguish a human apart from walls and other objects, and then follow the detected human even when new objects enter its field of vision.

**Our solution:**

Our strategy is to have the Neato “calibrate” on a person’s feet, creating a centroid that it follows around. Then when the person moves around, the scans of their feet will move. The tricky decision was figuring out how to maintain the object of interest on the feet once they moved. So we came up with a “closest centroid” solution. The Neato will collect a round of scans and then filter out all scans except the ten closest points to the centroid of the previous scan. In this way, we make sure that only the person’s feet will remain in subsequent scans, so even if new objects appear, as long as the person doesn’t move too drastically, their feet will still be the closest points to the previous centroid, allowing the Neato to distinguish between human and other objects.

**Code architecture:**

Our code uses OOP structure with a main Person_Follower class with an init method that initializes member variables (p-controller variables, centroid and filtering variables, and lidar readings), a processScans method that acts as the lidar subscriber callback and filters out scans based on the centroid, and an act method that applies a p-controller to the distance from the robot to the person based on filtered readings and publishes linear and angular velocities to the Neato. In our code’s main we have a while loop that runs the act method on repeat.

**Challenges:**

Our greatest challenge was figuring out how to distinguish feet from another object. Since we’re only using the Laser Scan as our primary sensor, we had to come up with a clever way to make it figure out that a specific cluster of points are the feet, and then in the next set of scans, when the feet have moved and the corresponding data points are different, that the Neato knows that a new cluster is still the feet. We solved this using the closest centroid solution described above.

**Possible improvements:**

The biggest flaw in our code was that the person must maintain a close distance to their previous location with each step for the robot to follow them. Since we use the closest points, if the person moves too drastically such that the person’s feet are no longer the closest points to the previous centroid, and say, a wall is nearby, it’s highly possible that the Neato will begin to follow the wall. And unfortunately, it seems that our parameters made it such that the rate at which a person could walk for the Neato to continue following him was slower than a normal walking pace.

Steps for improvement include fine-tuning the parameters in our code, which could potentially make the Neato move faster and be more responsive to drastic changes in the centroid and the difference between the successive feet scans would still be in a valid range. We could also consider adding in the camera and using that as a secondary sensor to check the position of the feet, or we could explore different ways of using the Laser Scans, such as projecting the scans into the Odom frame and then looking for motion that is unique to the feet.

**Key takeaways:**

The key takeaway is that one sensor is relatively limiting to the pursuit for this project. In this case, having a camera would’ve made things more robust. In addition, having an understanding of how much work the Neato can do for us and the limitations of certain approaches are good things to consider for next time.


## Obstacle Avoidance

![obstacle_avoidance](images/obstacle_avoidance.jpg?raw=true "Obstacle Avoidance")

**The problem:**

For this behavior, the Neato must be able to avoid any obstacle it encounters in front of it, such that it will never hit an object that blocks its path, potentially with the goal of reaching a specified point.

**Our solution:**
Our strategy included a large pivot. Our first approach was to try for the stretch goal and make the Neato move towards a point while avoiding any obstacles along the way. We explored the concept of potential fields and came up with the correct approach. Basically find a goal point in the Odom frame, and convert that into the Base-Link frame. Then point a vector from the Neato to the goal point, which is the preferred motion vector. Then have all nearby objects become repulsive forces on the Neato which create a net force vector that the Neato chooses to move in the direction and magnitude of. We managed to solve the repulsive forces and net force portions, but were stuck on the creating the goal point vector in the Base-Link frame. In the interest of time, we pivoted to the simpler approach; the Neato just moves forward, and when it encounters an obstacle in its path, it will either move left or right until it has side-stepped the object, and then continue to move forward in the original direction again.

**Code architecture:**

Our code uses OOP structure with a main ObstacleAvoidance class with an init method that initializes member variables (obstacle tracking variables, and lidar readings), a processScans method that acts as the lidar subscriber callback and determines the location of any obstacles in the Neato’s path, and an act method that changes between forward and turning states to either continue forward to side-step obstacles. In our code’s main we have a while loop that runs the act method on repeat.

**Challenges:**

Our  greatest challenge was mainly finding the goal point vector in the stretch goal. We managed to derive the correct approach, but by the time we did, it had already been 3 hours, and we realized that we would need another 3-5 hours before this one module was finished. So we decided to switch to the simpler approach to save time. The simpler approach didn’t have many problems, but it would’ve been cooler had we more time to work on the more robust method.

**Possible improvements:**

The simpler approach’s greatest flaw is that it really only works when there are things right in front of it that it must sidestep. The code logic is a bit more hard-coded, so it only avoids obstacles under certain conditions. For example, if the Neato encountered a corner of a wall, there’s a high probability that it just turn into the corner. In addition, the simpler approach just has the Neato blindly moving forward without any sort of goal, which makes it less complete.

The most logical step to improve would just be to implement our derivation of the potential fields method. It would’ve been possible if we had a couple more days, and we checked with other teams and confirmed that our theoretical solution was correct. It was really just a matter of time.

**Key takeaways:**

The key takeaway is that it’s good to know your own limitations and your own goals towards a project. We pushed a little too hard in a direction that we weren’t really ready to tackle, so we ended up using a lot of time towards a goal that was cool and thought-through but was never implemented. Learning proper scoping is a good idea.


## Finite State Control

![finite_state_controller](images/finite_state_controller.jpg?raw=true "Finite State Controller")
![finite_state_controller](images/fsm_diagram.png?raw=true "Finite State Diagram")

**The problem:**

For finite state control, the idea is to have the Neato be able to switch between different states and perform multiple behaviors autonomously, changing between the behaviors based on external stimuli.

**Our solution:**

Our approach for our finite state machine was to combine driving a square and person following. The two states were drive_square, where the Neato would continuously driving a square formation, and person_follow, where the Neato would follow a person around. The stimulus to change between states was whether or not the Neato detected something in front of it within the field of vision of the person_follow module. If it did, then it meant that was a person was in front, so it would begin to follow the person. Once the person jumped out of the way, the Neato would no longer detect anything, and thus would revert back to driving a square until another person stepped into its path. We decided to specifically use this pair of behaviors because it made logical sense out of the behaviors we had made previously. Wall following can’t continue indefinitely because at some point the Neato runs out of wall to follow, and our implementation of obstacle avoidance was done in a way that made it only work under specific conditions, so combining with another behavior was pragmatically unfeasible. Driving a square can continue as a passive state indefinitely, and the person follow state distinguishes itself  from the driving square state very clearly in that a person will enter the Neato’s field of vision. The tricky part was making sure that the Neato would not revert too early back to driving a square because the person got too far away from the Neato’s field of vision, and we had to make sure that while the Neato was driving a square it wouldn’t pick up any other objects as persons. We tuned parameters until we found a good balance between the two states.

**Code architecture:**

Our code uses OOP structure with a main FiniteStateController class with an init method that initializes member variables (state controller variables, and variables for the two behaviors described above), a processScans method that acts as the lidar subscriber callback and detects people in the Neato’s field of vision, and an act method that changes between the drive_square and person_follow states and implements the current behavior as described above. Another way we could’ve done this was to use ROS topics between our existing behavior scripts to communicate what state the Neato was in, but we deemed that just straight up combining the two scripts into one and then segregating them by conditionals was easier and had a higher chance of success. In our code’s main we have a while loop that runs the act method on repeat.

**Challenges:**

Our greatest challenge was balancing the detection between person_follow and drive_square. Often times during testing the Neato would be driving a square and then just detect a table leg or a wall, which meant that we needed to reduce the range of detection. However, if we reduced it too much, then during person following, the person being followed could only move in small baby steps very slowly because the range of the Neato was too small to detect a more comfortable walking pace. There was a lot of fine tuning that went into figuring out how to balance these two issues and ensure smooth and proper state transitions.

**Possible improvements:**

Our greatest flaw was that changing of states was based solely on whether something was detected in the Neato field of vision, which meant that anything that appeared in the range of detection would trigger a change of state. In that sense our detection for state change was too tenuous and delicate. In addition, while logical for a FSM pair, the driving square module is a little hard coded, so sometimes when switching back from person following, the Neato will just drive a square into a wall because it lost sense of its orientation.

Steps to improve include using a more robust behavior instead of drive_square or making drive_square a bit smarter such that it knew its orientation and wouldn’t crash into objects after leaving person_follow. At the same time more work could be done to make the detection of feet more robust, such that the Neato can continue to drive squares and detect objects as long as the objects aren’t feet. This could be done using motion detection in the Odom frame, or using a camera and sensor fusion.

**Key takeaways:**

The key takeaway is to remember that the Neato is not smart unless the programmer makes it so. There were definitely a couple of holes in this module that occurred because we weren’t thinking fully about what the Neato sees and what state transitions should look like. When creating more complex behavior patterns, recognizing how different behaviors interact with each other and taking that into account is a very useful lesson.
