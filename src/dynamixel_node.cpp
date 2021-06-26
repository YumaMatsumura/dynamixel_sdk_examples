/*
 $ rostopic pub -1 /set_joint_position std_msgs/Int16MultiArray "data: [512, 512, 512, 512]"

*/



#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/GetPosition.h"
#include <std_msgs/Int16MultiArray.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_MOVING_SPEED     32
#define ADDR_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE              1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler;
PacketHandler * packetHandler;

bool getPresentPositionCallback(
  dynamixel_sdk_examples::GetPosition::Request & req,
  dynamixel_sdk_examples::GetPosition::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int16_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read2ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint16_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req.id, position);
    res.position = position;
    return true;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return false;
  }
}

void setJointPositionCallback(const std_msgs::Int16MultiArray& msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result[6];
  uint16_t position[6];
  
  int num = msg.data.size();
  
  for(int i = 0; i < num; i++){
    dxl_comm_result[i] = COMM_TX_FAIL;
    //position[i] = (unsigned int)msg.data[i];
    position[i] = msg.data[i];
    dxl_comm_result[i] = packetHandler->write2ByteTxRx(portHandler, i+1, ADDR_GOAL_POSITION, position[i], &dxl_error);
    if(dxl_comm_result[i] != COMM_SUCCESS){
      ROS_ERROR("Failed to set position! Result: [ID:%d]", i+1);
    }
  }

  if (dxl_comm_result[0] == COMM_SUCCESS && dxl_comm_result[1] == COMM_SUCCESS && dxl_comm_result[2] == COMM_SUCCESS && dxl_comm_result[3] == COMM_SUCCESS, dxl_comm_result[4] == COMM_SUCCESS && dxl_comm_result[5] == COMM_SUCCESS) {
    ROS_INFO("setJointPosition : {%d, %d, %d, %d, %d, %d}", position[0], position[1], position[2], position[3], position[4], position[5]);
  }
}

void setJointSpeedCallback(const std_msgs::Int16MultiArray& msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result[6];
  uint16_t speed[6];
  
  int num = msg.data.size();
  
  for(int i = 0; i < num; i++){
    dxl_comm_result[i] = COMM_TX_FAIL;
    speed[i] = msg.data[i];
    dxl_comm_result[i] = packetHandler->write2ByteTxRx(portHandler, i+1, ADDR_MOVING_SPEED, speed[i], &dxl_error);
    if(dxl_comm_result[i] != COMM_SUCCESS){
      ROS_ERROR("Failed to set speed! Result: [ID:%d]", i+1);
    }
  }
  
  if (dxl_comm_result[0] == COMM_SUCCESS && dxl_comm_result[1] == COMM_SUCCESS && dxl_comm_result[2] == COMM_SUCCESS && dxl_comm_result[3] == COMM_SUCCESS, dxl_comm_result[4] == COMM_SUCCESS && dxl_comm_result[5] == COMM_SUCCESS) {
    ROS_INFO("setJointSpeed : {%d, %d, %d, %d, %d, %d}", speed[0], speed[1], speed[2], speed[3], speed[4], speed[5]);
  }
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }
  
  for(int i = 1; i < 7; i++){
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i);
      return -1;
    }
  }

  ros::init(argc, argv, "dynamixel_node");
  ros::NodeHandle nh;
  ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPositionCallback);
  ros::Subscriber set_joint_position_sub = nh.subscribe("/set_joint_position", 10, setJointPositionCallback);
  ros::Subscriber set_joint_speed_sub = nh.subscribe("/set_joint_speed", 10, setJointSpeedCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}
