
#include <sony_ipela/ptz_interface.h>

namespace sony_ipela 
{
  PtzInterface::PtzInterface(ros::NodeHandle &nh)
   : name_("sony_ipela_ptz_interface")
   , nh_(nh), pos_{0, 0}, vel_{0, 0}, eff_{0,0}
 { 
   ROS_INFO_STREAM_NAMED("sony_ipela", "Controller started");
    // Load rosparams
   nh.param("pan_joint", pan_joint_, std::string("joint_pan"));
   nh.param("tilt_joint", tilt_joint_, std::string("joint_tilt"));
   nh.param("camera_ip", camera_ip_, std::string("10.0.0.6"));
   nh.param("username", username_, std::string("admin"));
   nh.param("password", password_, std::string("password"));
  }

  void PtzInterface::init() {
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_pan_joint_(pan_joint_, &pos_[0], &vel_[0], &eff_[0]);
   jnt_state_interface_.registerHandle(state_handle_pan_joint_);

   hardware_interface::JointStateHandle state_handle_tilt_joint_(tilt_joint_, &pos_[1], &vel_[1], &eff_[1]);
   jnt_state_interface_.registerHandle(state_handle_tilt_joint_);

   registerInterface(&jnt_state_interface_);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_pan_joint_(jnt_state_interface_.getHandle(pan_joint_), &cmd_[0]);
   jnt_pos_interface_.registerHandle(pos_handle_pan_joint_);

   hardware_interface::JointHandle pos_handle_tilt_joint_(jnt_state_interface_.getHandle(tilt_joint_), &cmd_[1]);
   jnt_pos_interface_.registerHandle(pos_handle_tilt_joint_);

   registerInterface(&jnt_pos_interface_);
   ROS_INFO_STREAM_NAMED("sony_ipela", "Controller initialization done");
  }

  void PtzInterface::read() 
  {
    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    std::string read_url = "http://" + camera_ip_ + "/command/inquiry.cgi?inq=ptzf";

    curl = curl_easy_init();
  
    if(curl) {
    curl_easy_setopt(curl, CURLOPT_URL, read_url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    curl_easy_setopt(curl, CURLOPT_USERNAME, username_.c_str());
    curl_easy_setopt(curl, CURLOPT_PASSWORD, password_.c_str());
    res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    }

    //  Example return string from curl:
    //   AbsolutePanTilt=f71a,fb50&AbsolutePTZF=f71a,fb50,0000,2000

    std::vector<std::string> results;
    boost::split(results, readBuffer, boost::is_any_of("=,&"));

    // Store as 16bit twos complement 
    int16_t pan = std::stoul(results[1], nullptr, 16);
    int16_t tilt = std::stoul(results[2], nullptr, 16);
    
    const double to_rad = 0.00130899693899574718269;
   
    // Update joint value
    pos_[0] = pan * to_rad;
    pos_[1] = tilt * to_rad;

  }

  void PtzInterface::write() 
  {
  
    // Store command value in int16 for easy conversion to 16bit twos complement
    const double from_rad = 763.943726841097611690642064;
    int16_t pan = cmd_[0] * from_rad;
    int16_t tilt = cmd_[1] * from_rad;


    //  pan position: "F725" to "08DB"
    //  tilt position : "FB50" to "0190" (Eflip : off) (*5)
    //                  "FE70" to "04B0" (Eflip : on) (*5)
    //  speed: "1" to "24"
    
    //  pan
    // F725(h) -2267 = -170
    // 0000(h) = 0
    // 08DB(h) 2267  = 170

    // tilt eflip off       tilt eflip on
    // FB50(h) -1200 = -90        04B0(h) = 90
    // 0000(h) = 0          0000(h) = 0
    // 0190(h) 400 = 30         FE70(h) = -30



    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    std::stringstream ss;
    ss << "http://";
    ss << camera_ip_;
    ss << "/command/ptzf.cgi?AbsolutePanTilt=";
    ss << std::setfill('0') << std::setw(4) << std::hex << pan << "," << std::setfill('0') << std::setw(4) << std::hex << tilt << ",24";

    auto read_url = ss.str();

    curl = curl_easy_init();
  
    if(curl) {
    curl_easy_setopt(curl, CURLOPT_URL, read_url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    curl_easy_setopt(curl, CURLOPT_USERNAME, username_.c_str());
    curl_easy_setopt(curl, CURLOPT_PASSWORD, password_.c_str());
    res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    }


  }

  size_t PtzInterface::WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
  {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
  }

} // namespace
