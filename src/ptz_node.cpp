
#include <ros/ros.h>
#include <sony_ipela/ptz_interface.h>
#include <time.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ptz_controller");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Init ptz interface
  sony_ipela::PtzInterface ptz_interface(nh);
  ptz_interface.init();
  controller_manager::ControllerManager controller_manager(&ptz_interface, nh);
  // Set up timers
  ros::Rate period(5);
  ros::Duration elapsed_time;
  struct timespec last_time;
  struct timespec current_time;
  static const double BILLION = 1000000000.0;

  // Set last_time to current time for first round
  clock_gettime(CLOCK_MONOTONIC, &last_time);

  while(ros::ok()) {
    // Update time
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    last_time = current_time;

    // Update hardware
    ptz_interface.read();
    controller_manager.update(ros::Time::now(), elapsed_time);
    ptz_interface.write();
  
    period.sleep();
  }

}
  
