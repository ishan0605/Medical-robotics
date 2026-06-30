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
/**
 * @example echo_robot_state.cpp
 * An example showing how to continuously read the robot state.
 */


namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;

}

// The following global variables will store data required by functions and threads.
namespace global_variable{
  std::array< double, 16 > current_ee_pose; 
  bool flag_done_collecting; // Controls the point registration thread
  bool flag_done_moving; // Controls the entrp point registration thread
  std::vector<std::array< double, 16 >> collected_ee_poses; // Collected end effector poses
  std::vector<Eigen::Vector3d> left_points; // left_points for point registration
  std::vector<Eigen::Vector3d> right_points; // right_points for point registration
  std::vector<Eigen::Vector3d> nogozones; // Restricted zones defined in base frame coordinates
  std::vector<Eigen::Vector3d> nogozones_voxel; // Restricted zones defined in metric voxel coordinates

}

using namespace Eigen;
using namespace std;

/**
 * Calculate rotation matrix R and and the translation vector t from the given left and right points from
 * point registration. The code structure was the same as the python code provided in lecture slides
 * converted to c++.
 * 
 * @param points_in_left metric voxel coordinates of the registrations points
 * @param points_in_right base frame coordinates obtained from current franka end-effector pose.
 * @return pair<Matrix3d, Vector3d> Rotation matrix and translation vectors respectively
 */
pair<Matrix3d, Vector3d> R_and_t(const vector<Vector3d>& points_in_left, const vector<Vector3d>& points_in_right) {
    int num_points = points_in_left.size();
    int dim_points = points_in_left[0].size();

    MatrixXd left_mat(dim_points, num_points);
    MatrixXd right_mat(dim_points, num_points);

    for (int i = 0; i < num_points; ++i) {
        left_mat.col(i) = points_in_left[i];
        right_mat.col(i) = points_in_right[i];
    }

    Vector3d left_mean = left_mat.rowwise().mean();
    Vector3d right_mean = right_mat.rowwise().mean();
    MatrixXd left_M = left_mat.colwise() - left_mean;
    MatrixXd right_M = right_mat.colwise() - right_mean;

    Matrix3d M = left_M * right_M.transpose();
    JacobiSVD<MatrixXd> svd(M, ComputeFullU | ComputeFullV);
    Matrix3d R = svd.matrixV() * Matrix3d::Identity() * svd.matrixU().transpose();
    double det = R.determinant();
    Matrix3d V = svd.matrixV();
    if (det < 0)
        V.col(2) *= -1;
    R = V * Matrix3d::Identity() * svd.matrixU().transpose();
    Vector3d t = right_mean - R * left_mean;
    // Uncomment the following lines to print the rotation matrix and translation vector to the shell.
    /* std::cout << "ROTATION MATRIX" << std::endl;
    std::cout << R << std::endl;
    std::cout << "T-vector" << std::endl;
    std::cout << t << std::endl;*/
    return make_pair(R, t);
}

/**
 * Record voxel coordinates and base frame coordinates for n_poses poses and store them in global_variable::left_points
 * and global_variable::right_poses respectively. Update the gobal_variable::flag_done_collecting once done.
 * 
 * @param n_poses number of poses to record
 * @return std::vector<std::array< double, 16 >> 
 */
std::vector<std::array< double, 16 >> record_pose_thread(int n_poses=3){
  int collected_poses = 0; 
  std::string my_string = "";
  // Loop until n_poses have been collected
  while(collected_poses < n_poses){
    // Ask user to enter voxel coordinates of the point that is being registered. This can also be
    // stored in a file and then imported as well if that is more convenient.
    std::cout << "Enter the voxel coordinates of the point that you want to register: " << std::endl;
    std::string vox_x_str = "";
    std::string vox_y_str = "";
    std::string vox_z_str = "";
    std::cout << "x-coord: ";
    std::getline(std::cin, vox_x_str);
    std::cout << "y-coord: ";
    std::getline(std::cin, vox_y_str);
    std::cout << "z-coord: ";
    std::getline(std::cin, vox_z_str);
    double vox_x = stoi(vox_x_str)*0.008;
    double vox_y = stoi(vox_y_str)*0.008;
    double vox_z = stoi(vox_z_str)*0.0096;
    global_variable::left_points.push_back(Eigen::Vector3d(vox_x, vox_y, vox_z)); // store the voxel coords in left_points

    // Ask user to confirm when they are ready to register the base frame coordinate of the selected point.
    std::cout << "Press ENTER to collect the pose of the point, anything else to quit data collection" << std::endl;
    std::getline(std::cin, my_string);
    if(my_string.length()==0){
      // store the base frame coords to right_points
      global_variable::right_points.push_back(Eigen::Vector3d(global_variable::current_ee_pose[12], global_variable::current_ee_pose[13], global_variable::current_ee_pose[14]));      
      collected_poses++;
    }
    else{
      std::cout << "Exiting data collection"<<std::endl; 
      global_variable::flag_done_collecting = true; 
      break;
    }
  }
  // Update flag_done_collecting once done collecting.
  global_variable::flag_done_collecting = true; 
}
/**
 * Helper function to convert a metric voxel coordinate to base frame.
 * 
 * @param metric_position 
 * @param R_mat computed with R_and_t
 * @param t_vec computed with R_and_t
 * @return Eigen::Vector3d converted base frame coordinate
 */
Eigen::Vector3d voxel_to_base(Eigen::Vector3d metric_position, Eigen::Matrix3d R_mat, Eigen::Vector3d t_vec) {
  // Apply the formula discussed in lecture to convert voxel coordinate to base frame coordinate. y = R*x + t
  Eigen::Vector3d result = R_mat*metric_position + t_vec;
  return result;
}
/**
 * Helper function to calculate distance between two vectors.
 * 
 * @param v1 
 * @param v2 
 * @return double distance between v1 and v2
 */
double distance(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2){
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}
/**
 * Runs in a thread while collecting the entry point and update flag_done_moving when user presses ENTER.
 * 
 */
void move_to_entry_point(){
  std::string inp = "";
  std::cout << "Press ENTER to continue: ";
  std::getline(std::cin, inp);
  global_variable::flag_done_moving = true;
}
/**
 * Calculate the rotation matrix that will make the end-effector (needle) point in the same direction as
 * a desired vector (mainly the tumour location).
 * 
 * @param TP_minus_EP vector to point in the direction of
 * @return Eigen::Matrix3d rotation matrix to point in the direction as TP_minus_EP
 */
Eigen::Matrix3d rotation_matrix_to_point_to_vector(const Eigen::Vector3d& TP_minus_EP) {
    // Select the x-axis to be the same as the base frame's x-axis.
    Eigen::Vector3d x;
    x << 1,0,0;
    // Set the z-axis of the end-effector/needle to point towards TP_minus_EP.
    Eigen::Vector3d z = TP_minus_EP.normalized();
    // y-axis will be the cross product of z and x axes.
    Eigen::Vector3d y = z.cross(x).normalized();
    // Recalculate x-axis as the cross product of y and z axes.
    x = y.cross(z).normalized();
    // The columns of the rotation matrices are normalized vectors x, y, and z (by definition of a rotation matrix)
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = x;
    rotation_matrix.col(1) = y;
    rotation_matrix.col(2) = z;
    // Ensure the determinant is 1 and not -1.
    if (rotation_matrix.determinant() < 0) {
      y *= -1; // Flip the sign of y
      rotation_matrix.col(1) = y;
    }
    return rotation_matrix;
}
/**
 * Move the robot through the collected end-effector poses. Borrowed from the medcvr_point_collection file.
 * (Runs at the end once all the point registration and entry point selection is done)
 * 
 * @param robot Franka robot
 * @return auto 
 */
auto move_to_collected_points(franka::Robot& robot){
  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
  for (auto &ee_pose: global_variable::collected_ee_poses){
    Eigen::Matrix4d pose = Eigen::Matrix4d::Map(ee_pose.data());
    std::cout<<pose<<"\n"; 
    double time = 0.0;
    robot.control([&time, &pose, &ik_controller](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::JointVelocities {
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
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
}
/**
 * Create restriction zone specified by two opposite corners given by (x1,y1) and (x2,y2) on the given z-plane,
 * in voxel coordinates.
 * 
 * @param x1 
 * @param x2 
 * @param y1 
 * @param y2 
 * @param z 
 */
void create_blockage(int x1, int x2, int y1, int y2, int z) {
    for (int x = x1; x <= x2; ++x) {
        for (int y = y1; y <= y2; ++y) {
            global_variable::nogozones_voxel.push_back(Eigen::Vector3d(x, y, z));
        }
    }
}
/**
 * Remove restriction point specified by the voxel coordinate (x,y,z).
 *
 * @param x
 * @param y 
 * @param z 
 */
void remove_blockage(int x, int y, int z) {
    global_variable::nogozones_voxel.erase(std::remove_if(global_variable::nogozones_voxel.begin(), global_variable::nogozones_voxel.end(),
                                    [x, y, z](const Eigen::Vector3d& voxel) {
                                        return voxel[0] == x && voxel[1] == y && voxel[2] == z;
                                    }), global_variable::nogozones_voxel.end());
}
/**
 * Set the restriction zones in voxel coordinates. 
 * Notes: Currently the values are hardcoded but can be replaced with file input.
 */
void set_nogozones_voxel(){
  // Hardcode the nogozones (remove/add as needed)
    create_blockage(1, 4, 2, 9, 3);
    for (int z = 1; z <= 2; ++z) {
        create_blockage(1, 4, 0, 1, z);
        create_blockage(0, 1, 2, 5, z);
        create_blockage(-1, 0, 6, 9, z);
        create_blockage(1, 4, 8, 9, z);
        create_blockage(6, 7, 6, 9, z);
        create_blockage(4, 5, 2, 5, z);
    }
    remove_blockage(2, 6, 3);
    remove_blockage(2, 7, 3);
    remove_blockage(3, 6, 3);
    remove_blockage(3, 7, 3);    
}
/**
 * Set the nogozones converted in the base frame coordinates. Use voxel_to_base() with the given R_mat and t_vec.
 * 
 * @param R_mat 
 * @param t_vec 
 */
void set_nogozones(Eigen::Matrix3d R_mat, Eigen::Vector3d t_vec){
  for (Eigen::Vector3d zone:global_variable::nogozones_voxel){
      Eigen::Vector3d zone_metric(zone[0]*0.008, zone[1]*0.008, zone[2]*0.0096);
      global_variable::nogozones.push_back(voxel_to_base(zone_metric, R_mat, t_vec));
    }
}
/**
 * Find the point of intersection of vector v originated at point p with the given z-plane, using equation of
 * a line in 3d space.
 * 
 * @param v 
 * @param p 
 * @param z 
 * @return std::pair<double, double> (x, y) coordinates of the point of intersection on z-plane.
 */
std::pair<double, double> find_xy_given_z(const Eigen::Vector3d& v, const Eigen::Vector3d& p, double z) {
    double x0 = p[0], y0 = p[1], z0 = p[2];
    double v1 = v[0], v2 = v[1], v3 = v[2];

    double t = (z - z0) / v3;
    double x = x0 + t * v1;
    double y = y0 + t * v2;
    return {x, y};
}

int main(int argc, char** argv) {
  global_variable::flag_done_collecting = false; 

  std::vector<std::array< double, 16 >> ee_poses; 
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  //****************************************Break Point 1 - Record points******************************************************
  std::cout << "****BP1 - Record points****" << std::endl;
  // First Set restriction zones in voxel coordinates.
  set_nogozones_voxel();

  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
  try {
    franka::Robot robot(argv[1]);
    robotContext::robot = &robot;
    franka::Model model = robot.loadModel();
    robotContext::model = &model;
    int choice{}; 

    // Thread that records the points for point registration.
    std::thread t1(record_pose_thread, 4);
    // Set the robot to free motion until the points are registered.
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

    //****************************************BP2 - Find R and t******************************************************
    std::cout << "*****BP2 - Find R and t****" << std::endl;
    // Calculate R and t using the points registered.
    pair<Matrix3d, Vector3d> result = R_and_t(global_variable::left_points, global_variable::right_points);
    Eigen::Matrix3d R_mat = result.first;
    Eigen::Vector3d t_vec = result.second;

    //****************************************BP3 - Get TP (Tumour Point)*****************************************************
    std::cout << "***BP3 - Get tumour coords****" << std::endl;
    std::string vox_x_str = "";
    std::string vox_y_str = "";
    std::string vox_z_str = "";
    
    // Ask user to enter tumour point (TP) in voxel coordinates.
    std::cout << "Enter the voxel space coordinates of the tumour: " << std::endl;
    std::cout << "x-coord: ";
    std::getline(std::cin, vox_x_str);
    std::cout << "y-coord: ";
    std::getline(std::cin, vox_y_str);
    std::cout << "z-coord: ";
    std::getline(std::cin, vox_z_str);
    double vox_x = stoi(vox_x_str);
    double vox_y = stoi(vox_y_str);
    double vox_z = stoi(vox_z_str);
    
    // Convert voxel coordinates to base frame coordinates.
    std::array<double, 3> voxel_pos = {vox_x, vox_y, vox_z};
    Eigen::Vector3d metric_voxel;
    metric_voxel << voxel_pos[0]*0.008, voxel_pos[1]*0.008, voxel_pos[2]*0.0096;
    Eigen::Vector3d TP = voxel_to_base(metric_voxel, R_mat, t_vec);

    //************************************Set restriction zones in base frame***********************************
    // The restriction zones are hardcoded in the function right now but they can be imported from a file as well.    
    set_nogozones(R_mat, t_vec);

    //**************************************BP4 - Find an entry point*******************************************
    std::cout << "******BP4 - Find an entry point*****" << std::endl;
    bool nogo = true;
    Eigen::Vector3d TP_minus_EP;
    Eigen::Vector3d EP;
    // Loop the process of finding the entry point until a valid point is found. A valid entry point is that
    // from which the tumour is accessible (ie. the path from entry point(EP) to tumour point(TP) is not blocked 
    // by a restriction zone.
    while (nogo){
      global_variable::flag_done_moving = false; 
      std::thread t2(move_to_entry_point);
      // Robot free motion while an entry point is selected.
      try{
        robot.control([](const franka::RobotState& robot_state,
                        franka::Duration period) -> franka::Torques {

          franka::Torques output_torques = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
          global_variable::current_ee_pose =  robot_state.O_T_EE;
          if (global_variable::flag_done_moving)
            return franka::MotionFinished(output_torques);  
          else
            return output_torques;
        });
      }catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
      }
      t2.join();


      std::array<double, 3> entry_point =  {global_variable::current_ee_pose[12], global_variable::current_ee_pose[13], global_variable::current_ee_pose[14]};
      // Set the vector from base to EP
      EP = Eigen::Vector3d::Map(entry_point.data());
      // Set the UNIT vector from EP to TP
      TP_minus_EP = (TP-EP).normalized();

      // CHECKING IF PATH IS CLEAR, IF IT IS NOT CLEAR WE CAN LOOP BACK.
      nogo=false;
      for (int z = 0; z < 4; ++z) {
          Eigen::Vector3d z_vector(0,0,z*0.0096);
          double z_base = voxel_to_base(z_vector, R_mat, t_vec)[2];
          auto [x, y] = find_xy_given_z(TP_minus_EP, EP, z_base);
          // Get point of intersection between TP_minus_EP and z-plane
          Eigen::Vector3d intersection_point(x, y, z_base);
          // Check if the point is within the radius of any restriction zone.
          for (Eigen::Vector3d zone : global_variable::nogozones) {
              if (distance(zone, intersection_point) < 0.008) {
                  nogo = true;
                  std::cout << "NO GO!" << std::endl; // Inform user that they will hit a restricted zone (can also implement sound indication)
                  global_variable::flag_done_moving = false; // Set done_moving to false to allow collecting another entry point
                  break;
              }
          }
          if (nogo) {
            break;
          }
      }
      // If path is clear, break from the loop.
      if (!nogo){
        std::cout << "GOOD TO GO" << std::endl; 
        break;
      }
    }
  
    // Get the rotation matrix that points to the tumour (in the direction of TP_minus_EP)
    Eigen::Matrix3d rotation = rotation_matrix_to_point_to_vector(TP_minus_EP);



    // Add the EP and the desired rotation matrix to collected_ee_poses.
    Eigen::Matrix4d vox_to_base_trans = Eigen::Matrix4d::Identity();
    vox_to_base_trans.col(3).head(3) << EP;
    vox_to_base_trans.block<3,3>(0,0) << rotation;

    std::array<double, 16> vox_to_base_arr;
    int x = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            vox_to_base_arr[x] = vox_to_base_trans(j, i);
            x++;
        }
    }
    std::cout << vox_to_base_trans << std::endl;
    global_variable::collected_ee_poses.push_back(vox_to_base_arr);
    
    // Add the TP and the desired rotation matrix to collected_ee_poses.
    vox_to_base_trans.col(3).head(3) << TP;
    vox_to_base_trans.block<3,3>(0,0) << rotation;
    x = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            vox_to_base_arr[x] = vox_to_base_trans(j, i);
            x++;
        }
    }
    std::cout << vox_to_base_trans << std::endl;
    global_variable::collected_ee_poses.push_back(vox_to_base_arr);


  std::cout << "Done collecting ************** Moving to tumour" << std::endl;  

  // Move the end effector through the collected end effector poses.
  move_to_collected_points(*robotContext::robot);


  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
