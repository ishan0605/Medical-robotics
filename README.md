# Needle Insertion In A Brain Tumour Surgery

## Description
This project replicates, to an extent, a surgical robot system similar to ROSA ONE, which is utilized for minimally invasive brain surgery. Our goal is to insert a needle (or any similar tool such as suction needle or Da Vinci's clip appliers) to a surgically exposed tumour. It is designed to be used during the process of craniotomy while [removing a brain tumour](https://www.youtube.com/watch?v=ho2XWlJZDzg&t=2s).

My project is implementated through Frank Emika Panda Robot Arm in University of Toronto's Robotics Lab. Our code uses Eigen and libfranka libraries to perform algebraic computations of vectors and matrices and control the robot arm. We are using a 25cm long by 3mm thin needle for the extension to the Franka Hand. Our testing environment includes lego bricks to indicate the tumour location and setup the restricted zones around the tumour, and a skull mold to hold the lego setup.

One of the biggest challenges faced during the project was to perform the linear algebra calculations to compute the transformation matrices and direction vectors. After speaking with the Professor and the TAs, I was able to come up with a clever approach of setting the transformation matrix of the final poses. Another challenge faced during the development process was that robot access was required in order to test the code.

One of the things I would like to work on in the future is to implement this project on a ROS to improve development effeciency. Another small feature I would like to add was to add sound feedback when the robot detects the restricted zone (better than displaying the message on the console), that way the operator can focus on operating the robot rather than having to look back at the console everytime for indications.

Please find below instruction to configure the robot, run the code, and execute the project.

## Robot configuration
1. Open the Franka Desk app.
2. Go to Settings > End effector.
3. Select end effector as Franka Hand.
4. Add to the Mass and Transformation Matrix from Flange to End-Effector according to the mass and dimensions of the needle extension. (For our testing we used a 25 cm long needle and weighing about 10 grams).
5. Apply the changes.
6. Click on Homing to home the Franka Hand (fingers).
7. Attach the needle to the Franka Hand.
8. Setup the environment as outlined in the `instructions_and_testing.pdf`.
   
## Run the code
1. Ensure you have the required libraries and files given in the scratchpad/csc496 folder.
2. Open the terminal and cd to the csc496 folder.
3. Add `brain_insertion` to the `CMakeList.txt` to build and compile the file.
4. cd to build folder. If there is no build folder `mkdir build` then cd
5. `cmake ..`
6. `make`
7. Run the `run_brain_insertion` executable with the IP of the Franka Robot. For example: `./run_brain_insertion 192.168.1.107`
Note: Please refer to code documentation (comments) for further details about how the code works

## Interacting through the console.
**NOTE**: Ensure that the emergency switch is at-hand at all times to prevent any injuries to the operator, patient, or the robot.
1. Point registration (ensure to record the points with accuracy as it might affect the calculations and decisions made by the robot)
   * First, enter the x, y, and z voxel coordinates of the point to registration
   * Then move the needle tip to the point
   * Then press Enter to collect the base frame coordinate of the point
   * Repeat this for 3 other points (it is hardcoded right now but can later be changed to take input from a file)
2. Enter x, y, and z coordinates of the tumour in image/voxel space
3. Selecting a valid entry point
   * Move the robot to a desired entry point and press Enter
   * If the screen(console) displays "NO GO!", please select another entry point and press Enter.
   * Repeat until screen(console) displays "GOOD TO GO"
4. Wait till the needle insertion is complete and the code exits.

## Credits
Project by Ishan Gupta under supervision and support of Professor Lueder-Kahrs. 