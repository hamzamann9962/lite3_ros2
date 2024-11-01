#include <errno.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <netinet/in.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

using namespace std;
#define PI 3.1415926

#pragma pack(4)
/// @brief robot state structe implementation
struct RobotState {
  int robot_basic_state;                 ///< Basic motion state of robot
  int robot_gait_state;                  ///< Robot gait information
  double rpy[3];                         ///< IMU angular 
  double rpy_vel[3];                     ///< IMU angular velocity
  double xyz_acc[3];                     ///< IMU acceleration
  double pos_world[3];                   ///< Position of robot in world coordinate system
  double vel_world[3];                   ///< The speed of robot in the world coordinate system
  double vel_body[3];                    ///< Speed of robot in body coordinate system
  unsigned touch_down_and_stair_trot;    ///< This function has not been activated for the time being. This data is only used to occupy the position
  bool is_charging;                      ///< Not opened temporarily
  unsigned error_state;                  ///< Not opened temporarily
  int robot_motion_state;                ///< Robot action status
  double battery_level;                  ///< Battery Percentage
  int task_state;                        ///< Not opened temporarily
  bool is_robot_need_move;               ///< When the robot is standing in place, whether it is pushed to switch into motion mode
  bool zero_position_flag;               ///< Zero return flag bit
  double ultrasound[2];
};
struct RobotStateReceived {
  int code;                              ///< Command code  
  int size;                              ///< Command value                             
  int cons_code;                         ///< Command type
  struct RobotState data;
};

/// @brief Jonint state structe implementation
struct JointState {
  double LF_Joint;
  double LF_Joint_1;
  double LF_Joint_2;
  double RF_Joint;
  double RF_Joint_1;
  double RF_Joint_2;
  double LB_Joint;
  double LB_Joint_1;
  double LB_Joint_2;
  double RB_Joint;
  double RB_Joint_1;
  double RB_Joint_2;
};
struct JointStateReceived {
  int code;
  int size;
  int cons_code;
  struct JointState data;
};

/// @brief handlestate state structe implementation
struct handleState {
  double left_axis_forward;              ///< Left rocker y-axis,        range: - 1~1
  double left_axis_side;                 ///< Left rocker x-axis,         range: - 1~1
  double right_axis_yaw;                 ///< right rocker y-axis,        range: - 1~1
  double goal_vel_forward;               ///< Target linear speed in x direction
  double goal_vel_side;                  ///< Target linear speed in y direction
  double goal_vel_yaw;                   ///< Target Yaw angular velocity
};
struct handleStateReceived {
  int code;
  int size;
  int cons_code;
  struct handleState data;
};

int main(int argc, char **argv) {

  // Initialize the ros node
  ros::init(argc, argv, "qnx2ros");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  int server_port;
  int defaul_port =43897;
  private_nh.param<int>("server_port", server_port, defaul_port);
  int filter_size;
  private_nh.param<int>("filter_size", filter_size, 1);
  bool is_vel_world;
  private_nh.param<bool>("is_vel_world", is_vel_world, true);
  tf::TransformBroadcaster odom_broadcaster;

  //Define ros topics
  ros::Publisher leg_odom_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("leg_odom", 1);    
  ros::Publisher leg_odom_pub2 = nh.advertise<nav_msgs::Odometry>("leg_odom2", 1);
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 1);
  ros::Publisher handle_pub = nh.advertise<geometry_msgs::Twist>("/handle_state", 1);
  ros::Publisher ultrasound_pub = nh.advertise<std_msgs::Float64>("/us_publisher/ultrasound_distance", 1);

  //Set related configuration of udp qnx2ros
  int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0) 
  {
    perror("socket");
    exit(1);
  }
  struct sockaddr_in addr_serv;
  memset(&addr_serv, 0, sizeof(struct sockaddr_in));        ///< initialize to zeros
  addr_serv.sin_family = AF_INET;                           ///< host byte order
  addr_serv.sin_port = htons(server_port);                    ///< port in network byte order
  addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);            ///< automatically fill in my IP
  int len = sizeof(addr_serv);
  if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0) 
  {
    perror("bind error:");
    exit(1);
  }
  int recv_num = -1;
  char recv_buf[500];

  ros::Rate loop_rate(10000);
  
  //Verify the received upd datagram,Receive odometer data and turn it into ros topic for publish
  int i=0;
  while (ros::ok())
  {
    if ((recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0,(struct sockaddr *)&addr_serv,
                            (socklen_t *)&len)) < 0){
      perror("recvfrom error:");
      exit(1);
    }
    
    i++;
    if(i==3000)   
    {
      ROS_INFO("qnx2ros received: [%d] bytes",recv_num);
      i=0;
    }

    //Processing robot status information from action host
    if (recv_num == sizeof(RobotStateReceived)) 
    {
      RobotStateReceived *dr = (RobotStateReceived *)(recv_buf);
      RobotState *robot_state = &dr->data;

      if (dr->code == 2305) 
      {
        geometry_msgs::PoseWithCovarianceStamped leg_odom_data;               
        leg_odom_data.header.frame_id = "odom";                    
        leg_odom_data.header.stamp = ros::Time::now();
        leg_odom_data.pose.pose.orientation =
            tf::createQuaternionMsgFromYaw(robot_state->rpy[2] / 180 * PI);
        leg_odom_data.pose.pose.position.x = robot_state->pos_world[0];
        leg_odom_data.pose.pose.position.y = robot_state->pos_world[1];
        leg_odom_data.pose.pose.position.z = robot_state->pos_world[2];
        leg_odom_pub.publish(leg_odom_data);

        nav_msgs::Odometry leg_odom_data2;        
        leg_odom_data2.pose = leg_odom_data.pose;
        leg_odom_data2.twist.twist.linear.x = robot_state->vel_body[0];
        leg_odom_data2.twist.twist.linear.y = robot_state->vel_body[1];
        leg_odom_data2.twist.twist.angular.z = robot_state->rpy_vel[2];
        leg_odom_pub2.publish(leg_odom_data2);
        
        sensor_msgs::Imu imu_msg;     
        imu_msg.header.frame_id = "imu";
        imu_msg.header.stamp = ros::Time::now();
        auto q = tf::createQuaternionFromRPY(robot_state->rpy[0] / 180 * PI,
                                            robot_state->rpy[1] / 180 * PI,
                                            robot_state->rpy[2] / 180 * PI);
        tf::quaternionTFToMsg(q, imu_msg.orientation);
        imu_msg.angular_velocity.x = robot_state->rpy_vel[0];
        imu_msg.angular_velocity.y = robot_state->rpy_vel[1];
        imu_msg.angular_velocity.z = robot_state->rpy_vel[2];
        imu_msg.linear_acceleration.x = robot_state->xyz_acc[0];
        imu_msg.linear_acceleration.y = robot_state->xyz_acc[1];
        imu_msg.linear_acceleration.z = robot_state->xyz_acc[2];
        imu_pub.publish(imu_msg);

        std_msgs::Float64 ultrasound_distance;
        ultrasound_distance.data = robot_state->ultrasound[1];
        ultrasound_pub.publish(ultrasound_distance);
        
        geometry_msgs::TransformStamped leg_odom_trans;       
        leg_odom_trans.header.stamp = ros::Time::now();
        leg_odom_trans.header.frame_id = "odom";
        leg_odom_trans.child_frame_id = "base_link";
        leg_odom_trans.transform.translation.x = leg_odom_data.pose.pose.position.x;
        leg_odom_trans.transform.translation.y = leg_odom_data.pose.pose.position.y;
        leg_odom_trans.transform.translation.z = leg_odom_data.pose.pose.position.z;
        leg_odom_trans.transform.rotation = imu_msg.orientation;
        odom_broadcaster.sendTransform(leg_odom_trans);
      }
    } 
    
    //Processing Joint status information from action host
    if (recv_num == sizeof(JointStateReceived)) 
    {
      JointStateReceived *dr = (JointStateReceived *)(recv_buf);
      JointState *joint_state = &dr->data;
      if (dr->code == 2306)
      {
        sensor_msgs::JointState joint_state_data;
        joint_state_data.header.stamp = ros::Time::now();
        joint_state_data.name.resize(12);
        joint_state_data.position.resize(12);

        joint_state_data.name[0] = "LF_Joint";
        joint_state_data.position[0] = -joint_state->LF_Joint;
        joint_state_data.name[1] = "LF_Joint_1";
        joint_state_data.position[1] = -joint_state->LF_Joint_1;
        joint_state_data.name[2] = "LF_Joint_2";
        joint_state_data.position[2] = -joint_state->LF_Joint_2;

        joint_state_data.name[3] = "RF_Joint";
        joint_state_data.position[3] = -joint_state->RF_Joint;
        joint_state_data.name[4] = "RF_Joint_1";
        joint_state_data.position[4] = -joint_state->RF_Joint_1;
        joint_state_data.name[5] = "RF_Joint_2";
        joint_state_data.position[5] = -joint_state->RF_Joint_2;

        joint_state_data.name[6] = "LB_Joint";
        joint_state_data.position[6] = -joint_state->LB_Joint;
        joint_state_data.name[7] = "LB_Joint_1";
        joint_state_data.position[7] = -joint_state->LB_Joint_1;
        joint_state_data.name[8] = "LB_Joint_2";
        joint_state_data.position[8] = -joint_state->LB_Joint_2;

        joint_state_data.name[9] = "RB_Joint";
        joint_state_data.position[9] = -joint_state->RB_Joint;
        joint_state_data.name[10] = "RB_Joint_1";
        joint_state_data.position[10] = -joint_state->RB_Joint_1;
        joint_state_data.name[11] = "RB_Joint_2";
        joint_state_data.position[11] = -joint_state->RB_Joint_2;
        joint_state_pub.publish(joint_state_data);
      }
    } 
    
    //Processing handle status information from app
    if (recv_num == sizeof(handleStateReceived))
    {
      handleStateReceived *dr = (handleStateReceived *)(recv_buf);
      handleState *handle_state = &dr->data;
      if (dr->code == 2309) 
      {
        geometry_msgs::Twist handle_state_msg;
        handle_state_msg.linear.x = handle_state->left_axis_forward;
        handle_state_msg.linear.y = handle_state->left_axis_side;
        handle_state_msg.angular.z = - handle_state->right_axis_yaw;
        handle_pub.publish(handle_state_msg);    
      }
    }
    loop_rate.sleep();
  }
  close(sock_fd);
  return 0;
}
