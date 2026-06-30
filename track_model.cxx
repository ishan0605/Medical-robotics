/*=========================================================================

  Program:   OpenIGTLink -- Example for Tracker Server Program
  Language:  C++

  Copyright (c) Insight Software Consortium. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <array>
#include <iostream>
#include <cstdlib>

#include <type_traits>
#include <zmq.hpp>
#include <iostream>
#include <thread>

#include "igtlOSUtil.h"
#include "igtlTransformMessage.h"
#include "igtlServerSocket.h"


namespace global_vars {
  std::array<double, 16> recv_pose;
}

/**
 * @brief recv_poses function receives the poses 
 *
 * This function receives the poses from the desktop and stores them in the global variable
 */
void recv_poses() {
  zmq::context_t ctx;
  zmq::socket_t z_socket(ctx, zmq::socket_type::sub);
  z_socket.connect("tcp://192.168.0.3:8081");
  z_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

  while (true) {
    zmq::message_t poseMsg;

    z_socket.recv(&poseMsg);
    int numValues = poseMsg.size() / sizeof(double);
    std::vector<double> pose;
    for(int i = 0; i < numValues; i++) {
        pose.push_back(*reinterpret_cast<double*>(poseMsg.data()+i*sizeof(double)));
    }
    std::array<double, 16> poseArray;
    for (int i = 0; i < 16; i++) {
        poseArray[i] = pose[i];
    }
    global_vars::recv_pose = poseArray;
  }
}

void GetRandomTestMatrix(igtl::Matrix4x4& matrix);

int main(int argc, char* argv[])
{
  //------------------------------------------------------------
  // Parse Arguments

  if (argc != 3) // check number of arguments
    {
    // If not correct, print usage
    std::cerr << "Usage: " << argv[0] << " <port> <fps>"    << std::endl;
    std::cerr << "    <port>     : Port # (18944 in Slicer default)"   << std::endl;
    std::cerr << "    <fps>      : Frequency (fps) to send coordinate" << std::endl;
    exit(0);
    }

  int    port     = atoi(argv[1]);
  double fps      = atof(argv[2]);
  int    interval = (int) (1000.0 / fps);

  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetDeviceName("Tracker");

  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int r = serverSocket->CreateServer(port);


  if (r < 0)
    {
    std::cerr << "Cannot create a server socket." << std::endl;
    exit(0);
    }

  std::thread t1(recv_poses);
  std::cout << "made a connection" << std::endl;
  igtl::Socket::Pointer socket;
  while (1)
    {
    //------------------------------------------------------------
    // Waiting for Connection
    socket = serverSocket->WaitForConnection(1000);
    
    if (socket.IsNotNull()) // if client connected
      {
      //------------------------------------------------------------
      // loop
      int i = 0;
      while (true)
        {
        igtl::Matrix4x4 matrix;
        std::cout << "Asked to send transform" << std::endl;
        GetRandomTestMatrix(matrix);
        transMsg->SetDeviceName("Tracker");
        transMsg->SetMatrix(matrix);
        transMsg->Pack();
        socket->Send(transMsg->GetPackPointer(), transMsg->GetPackSize());
        igtl::Sleep(interval); // wait
        }
      }
    }
  
  t1.join();
  //------------------------------------------------------------
  // Close connection (The example code never reachs to this section ...)
  
  socket->CloseSocket();

}


void GetRandomTestMatrix(igtl::Matrix4x4& matrix)
{
  std::array<double, 16> pose;
  pose = global_vars::recv_pose;

  for (int i = 0; i < 16; i++) {
      matrix[i % 4][i / 4] = pose[i];
  }

  // convert to millimeters
  matrix[0][3] *= 1000;
  matrix[1][3] *= 1000;
  matrix[2][3] *= 1000;

  igtl::PrintMatrix(matrix);
}

