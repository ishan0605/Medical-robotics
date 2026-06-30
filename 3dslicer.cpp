/**
 * @file final_project.cpp
 * @brief This file contains the implementation of a final project.
 *
 * This file contains the complete implementation of the final project for CSC496H5.
 * 
 * The project's main aim is to connect the actual Frank Emika Panda robot with the 3D slicer
 * and perform the following tasks:
 *      - Track the end-effector poses of the robot in the 3D slicer
 *      - Perform point registration to align the robot poses with the image space points of the 3D slicer
 *      - Reach a point or set of points on the Lego board by sending values from the 3D slicer
 *
 * @author Ishan Gupta and Jaimin Patel
 * @date July 18th 2024
 */

// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
 #include <franka/exception.h>
#include <franka/robot.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <thread>
#include "ik.h"
#include <chrono>
#include <zmq.hpp>

/**
 * This namespace contains information about the robot, gripper and model.
 */ 
namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;
}

namespace global_vars {
  std::array< double, 16 > current_ee_pose; 
  std::array<double, 16> recv_pose;
  Eigen::Matrix3d golbal_r;
  Eigen::Vector3d golbal_t;
  bool done_playing;
}

namespace global_variable{
  std::array< double, 16 > current_ee_pose; 
  bool flag_done_collecting; 
  std::vector<std::array< double, 16 >> collected_ee_poses;
}

zmq::context_t ctx; 
zmq::socket_t z_socket(ctx, zmq::socket_type::sub); // Subscriber socket (gets the poses from slicer)
zmq::socket_t z_socket1(ctx, zmq::socket_type::pub); // Publisher socket (sends the poses to slicer)

/**
 * @brief recv_poses function receives the poses 
 *
 * This function receives the poses from the 3D slicer. The function will receive the poses
 * and store them in a global variable. This function can receive at most 5 poses. The global
 * variable is used to store the poses is of the size 16. The first element of the array is the
 * number of poses and the rest of the elements are the poses.
 */
void recv_poses() {
  z_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0); //
  zmq::message_t poseMsg;
  std::cout << "waiting for input from 3D slicer (5 maximum)" << std::endl;
  z_socket.recv(&poseMsg);
  std::cout << "got result" << std::endl;
  int numValues = poseMsg.size() / sizeof(double);  // calculate the number of values in the message
  std::vector<double> pose;
  for(int i = 0; i < numValues; i++) { 
    pose.push_back(*reinterpret_cast<double*>(poseMsg.data()+i*sizeof(double)));
  }
  std::array<double, 16> poseArray; // The pose array to store the pose (at most 5 poses) (The size of array if number of max poses + 1)
  poseArray[0] = numValues / 3; // The first element of the array is the number of poses
  for (int i = 0; i < numValues; i++) {  // Adding the poses to the array
    poseArray[i+1] = pose[i];
    std::cout << pose[i];
  }
  global_vars::recv_pose = poseArray;

}
 

/**
 * @brief record_pose_thread function records the current pose of the robot
 *
 * This function uses the robot state to record the current pose of the robot. The
 * function will record the pose and storing it in a vector n_poses times. The function
 * will stop collecting poses if the user presses anything other than ENTER.
 * 
 * @param n_poses the number of poses to collect
 *
 * @return std::vector<std::array< double, 16 >> a vector of 4x4 matrices representing the poses
 */
std::vector<std::array< double, 16 >> record_pose_thread(int n_poses=3){
  int collected_poses = 0; 
  std::string my_string = "";
  
  while(collected_poses < n_poses){
    std::cout << "Press ENTER to collect current pose, anything else to quit data collection" << std::endl;
    std::getline(std::cin, my_string);
    if(my_string.length()==0){
      global_variable::collected_ee_poses.push_back(global_variable::current_ee_pose);
      std::cout << Eigen::Matrix4d::Map(global_variable::current_ee_pose.data()) << std::endl;
      collected_poses++;
    } else {
      std::cout << "Exiting data collection"<<std::endl; 
      global_variable::flag_done_collecting = true; 
      break;
    }
  }
  global_variable::flag_done_collecting = true; // Set the flag to true to stop the robot control loop
 
}
 
/**
 * @brief this function calculates the distance between two points
 *
 * This function calculates the distance between current 3D position and the desired 3D position
 * using the formula sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2).
 * 
 * @param v1 the first point
 * @param v2 the second point
 *
 * @return the distance between the two points
 */
double distance(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2){
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}

/**
 * @brief Computes the optimal rigid transformation (rotation and translation) to align two sets of 3D points.
 *
 * This function implements a point set registration algorithm that computes the optimal
 * rigid transformation (rotation and translation) to align a source point set to a target
 * point set. It uses Singular Value Decomposition (SVD) to calculate the rotation matrix
 * and then computes the translation vector. The algorithm assumes that the point sets are
 * already centered at their respective means.
 *
 * @param X The source point set, a matrix with each column representing a 3D point.
 * @param Y The target point set, a matrix with each column representing a 3D point.
 * @param R The computed rotation matrix (output).
 * @param t The computed translation vector (output).
 */
void point_register(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y, Eigen::Matrix3d& R, Eigen::Vector3d& t) {
    // Subtract the mean from each column
    Eigen::MatrixXd Xc = X.colwise() - X.rowwise().mean();
    Eigen::MatrixXd Yc = Y.colwise() - Y.rowwise().mean();
 
    // Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Xc * Yc.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
 
    // Compute the rotation matrix
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(X.rows(), Y.rows());
    if ((V * U.transpose()).determinant() < 0) {
        S(X.rows() - 1, Y.rows() - 1) = -1;
    }
    R = V * S * U.transpose();
 
    // Compute the translation vector
    t = Y.rowwise().mean() - R * X.rowwise().mean();
}

std::vector<std::array< double, 16 >> get_poses(){
  std::cout << "press anything to stop" << std::endl;
  std::cin.ignore();
  global_vars::done_playing = true;
}

/**
 * @brief send_poses function sends the poses to the 3D slicer
 * 
 * This function is used to send tracking data to the 3D slicer. The 3D slicer 
 * has a subscriber socket that listens to the tracking data. The function sends
 * transformation matrices to the 3D slicer to help visualize real-time the 
 * end-effector poses of the robot.
 */
void send_poses() {
  Eigen::Vector3d pos;
  Eigen::Vector3d sl_pos;
  Eigen::Matrix3d sl_rot;
  Eigen::Matrix3d r_rot;
  std::array< double, 16 > rot;

  while(true) {
    rot = global_variable::current_ee_pose;
    r_rot << rot[0], rot[3], rot[6],
            rot[1], rot[4], rot[7],
            rot[2], rot[5], rot[8];
    sl_rot = r_rot * global_vars::golbal_r.inverse();
    for (int i = 0; i < 9; i++) {
      rot[i] = sl_rot(i / 3, i % 3);
    }

    pos << rot[12], rot[13], rot[14];
    sl_pos = global_vars::golbal_r.inverse() * (pos - global_vars::golbal_t);
    rot[12] = sl_pos[0];
    rot[13] = sl_pos[1];
    rot[14] = sl_pos[2];
    zmq::message_t jointAnglesMessage(rot);
    z_socket1.send(jointAnglesMessage, zmq::send_flags::dontwait);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

/**
 * This main function is used to control the robot to perform the overall task of 
 * tracking the end-effector poses of the robot and perfroming the point registration
 * algorithm to align the robot poses with the image space points.
 *
 * STEP 1: Set up the sockets for communication with the 3D slicer
 *         This includes both the subscriber and publisher sockets.
 *
 * STEP 2: Record the poses of the robot in physical space for point registration.
 *
 * STEP 3: Perform the point registration algorithm to align the robot poses with the image space points.
 * 
 * STEP 4: Give the user the option to either experiment with the robot and see how the robot end-effector 
 *         moves in 3D-slicer or reach a point/points on the Lego board.
 */
int main(int argc, char** argv) {
  global_variable::flag_done_collecting = false; 
 
  std::vector<std::array< double, 16 >> ee_poses; 
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  z_socket.bind("tcp://*:8080");
  z_socket1.bind("tcp://*:8081");

  

  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
  try {
    franka::Robot robot(argv[1]);
    robotContext::robot = &robot;
    franka::Model model = robot.loadModel();
    robotContext::model = &model;
    int choice{}; 
    
    std::thread t1(record_pose_thread, 5);
    try{
      robot.control([](const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::Torques {
 
        franka::Torques output_torques = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
        global_variable::current_ee_pose =  robot_state.O_T_EE;
        if (global_variable::flag_done_collecting)
          return franka::MotionFinished(output_torques);  
        else
          return output_torques;
      });
    }catch (franka::Exception const& e) {
      std::cout << e.what() << std::endl;
      return -1;
    }
    t1.join();
 
  std::cout << "Done collecting" << std::endl;
 
 
  // registration
  double dis = 0.008;
  // Define your data points here for X and Y
  Eigen::MatrixXd X(4, 3); // Initialize a 3x3 matrix for X 
  Eigen::MatrixXd Y(4, 3); // Initialize a 3x3 matrix for Y
 
  //image space points
  X << 58.967041, 19.767744, 3.1,
       27.631956, -4.760903, 3.1,
       -20.175245, -4.296045, 3.1,
      3.47056, 51.547157, 3.1;
  
  X = X * 0.001;
  // physical space points
  Y << global_variable::collected_ee_poses[0].data()[12], global_variable::collected_ee_poses[0].data()[13], global_variable::collected_ee_poses[0].data()[14],
      global_variable::collected_ee_poses[1].data()[12], global_variable::collected_ee_poses[1].data()[13], global_variable::collected_ee_poses[1].data()[14],
      global_variable::collected_ee_poses[2].data()[12], global_variable::collected_ee_poses[2].data()[13], global_variable::collected_ee_poses[2].data()[14],
      global_variable::collected_ee_poses[3].data()[12], global_variable::collected_ee_poses[3].data()[13], global_variable::collected_ee_poses[3].data()[14];



  std::cout << "X:" << std::endl << X << std::endl;
  std::cout << "Y:" << std::endl << Y << std::endl;
  
  // These will hold the output rotation matrix and translation vector
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
 
  // Call the function with your data
  point_register(X.transpose(), Y.transpose(), R, t);

  // Store the rotation matrix and translation vector in global variables
  global_vars::golbal_r = R;
  global_vars::golbal_t = t;
  
  // Output the results - This is important for making sure if the values are correct and within the expected range for safety reasons
  std::cout << "Rotation Matrix R:" << std::endl << R << std::endl;
  std::cout << "Translation Vector t:" << std::endl << t << std::endl;


  global_variable::current_ee_pose = robot.readOnce().O_T_EE;
  // std::thread t0(get_poses, &robot);
  std::thread t2(send_poses);
  
  while(true){
    std::string question;
    std::cout << "Press 1 to experiment or anything else to reach some point on Lego board!"<<std::endl;
     std::getline(std::cin, question);
    if(question == "1"){
      try {
        franka::Robot robot(argv[1]);
        robotContext::robot = &robot;
        // franka::Model model = robot.loadModel();
        // robotContext::model = &model;
        int choice{}; 
 
        global_vars::done_playing = false;
        std::thread t0(get_poses);
        try{
            robot.control([](const franka::RobotState& robot_state,
                        franka::Duration period) -> franka::Torques {
 
            franka::Torques output_torques = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
            global_variable::current_ee_pose =  robot_state.O_T_EE;
            if (global_vars::done_playing)
                return franka::MotionFinished(output_torques);  
            else
                return output_torques;
            });
        }catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
            return -1;
        }
        t0.join();
    } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
  }
  std::string yoo = "";
  std::cout << "Press select points in 3D slicer and then press ENTER." << std::endl;
  std::getline(std::cin, yoo);
  recv_poses(); 

  std::cout << "\n" << std::endl;
  std::cout << global_vars::recv_pose[0] << std::endl;
  for (int num_p = 0; num_p < global_vars::recv_pose[0] * 3; num_p += 3){
  // Now i want to find the Transformation matrix for voxel position (4, 4, 0)
  Eigen::Vector3d voxel;
  voxel << global_vars::recv_pose[1 + num_p],global_vars::recv_pose[2 + num_p],global_vars::recv_pose[3 + num_p];
  std::cout << " Voxel:" << std::endl << voxel << std::endl;

  Eigen::Vector3d f = R * (voxel*0.001) + t;
  std::cout << "final Vector f:" << std::endl << f << std::endl;
 
  Eigen::Matrix4d pose;

 
  Eigen::Matrix3d i;
  i << 0.727379,  -0.685278,   0.036248,
      -0.686139,  -0.727147,  0.0216767,
      0.0115031, -0.0406383,  -0.999108;
  pose << i, f,
        0,0,0,1;
  double time = 0.0;
  try{
  // t0.join();
  robot.control([&time, &pose, &ik_controller](const franka::RobotState& robot_state,
                                        franka::Duration period) -> franka::JointVelocities {

    global_variable::current_ee_pose =  robot_state.O_T_EE;
    time += period.toSec();
    franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
    Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
    Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
    double dist = distance(current_position, desired_position);
 
    if (time >= 15.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005) ) {
      output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
      return franka::MotionFinished(output_velocities);
    }
    
    return output_velocities;
  });
 
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }}
  }
  }catch (franka::Exception const& e) {
      std::cout << e.what() << std::endl;
      return -1;
    }
 
  return 0;
}
