#ifndef SONY_IPELA_PTZ_INTERFACE_H
#define SONY_IPELA_PTZ_INTERFACE_H

// C++
#include <curl/curl.h>
#include <string>
#include <sstream> 

// Boost
#include <boost/algorithm/string.hpp>

// ROS
#include <ros/ros.h>

// ROS Control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>


namespace sony_ipela 
{

/// \brief Interface for controling and Sony Ipela camera
class PtzInterface : public hardware_interface::RobotHW
{
public:

  /**
    * \brief Constructor
    * \param nh - Node handle for topics
    */
  PtzInterface(ros::NodeHandle &nh);

  /** \brief Initialize the hardware interface */
  void init();

  /* \brief Read camera position. */
  void read();

  /* \brief Write command to camera */
  void write();

private:
  
  ros::NodeHandle nh_;
  std::string name_;

  // Hardware interfaces
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  // Commands
  double cmd_[2];

  // States
  double pos_[2];
  double vel_[2];
  double eff_[2];

  // Camera settings
  std::string camera_ip_;
  std::string pan_joint_;
  std::string tilt_joint_;
  std::string username_;
  std::string password_;

  /** \breif Helper for libcurl */
  static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);
}; // class

} // namespace

#endif //SONY_IPELA_PTZ_INTERFACE_H

