# Webots_Bug0_Implementation

UNC Comp 581 - Introduction to Robotics

Bug0 plath planning implementation final project. Successfully navigates through obstacles and includes backwards traversal if the robot gets stuck on an unfamiliar shaped object. The algorithm works like this: 1. Head straight towards goal until robot encounters an object, 2. Turn right and traverse object, 3. Once robot reaches closest point to the goal, head straight towards goal, 4. Stop when reaching the goal

The robot's orientation vector is computed by subtracting the robots current coordinates from its previous coordinates. The angle between the robot's orientation and the direct path to the goal point can be computed using a combination of vector computations and trigonometry functions. This angle is then used as an error for a proportional control method. This simple yet effective methodology is used to implement the bug0 algorithm with built in backup features to ensure the robot can get out of any sticky situations. To run this code you have to download the webots emulator software and python 3.7. 
