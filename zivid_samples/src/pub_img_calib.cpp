#include <zivid_camera/Settings2DAcquisitionConfig.h>
#include <zivid_camera/Capture2D.h>
#include <zivid_camera/Capture.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/SetBool.h>
#include <cv_bridge/cv_bridge.h>
#include <zivid_camera/LoadSettingsFromFile.h>

#include <ros/ros.h>
#include <ros/package.h>

#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

namespace
{
const ros::Duration default_wait_duration{ 30 };

bool flag_pub_ = false;
bool flag_pub_info_ = false;
sensor_msgs::Image::ConstPtr  msg_img_;
sensor_msgs::CameraInfo msg_info_;
ros::ServiceClient load_settings_;

// void capture()
// {
//   ROS_INFO("Calling capture_2d service");
//   zivid_camera::Capture2D capture_2d;
//   CHECK(ros::service::call("/zivid_camera/capture_2d", capture_2d));
// }
void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

void on_image_color(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("2D color image received");
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr   = cv_bridge::toCvCopy(msg, "rgb8");                                            // Transforming image encoding to rgb8
  msg_img_ = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_ptr->image).toImageMsg();
  
  flag_pub_ = true;
}

void on_info_color(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  ROS_INFO("2D color INFO received");
  msg_info_ = *msg;
  
  flag_pub_info_ = true;
}

}  // namespace

bool callback_zivid_pub_img(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if(req.data)
  {
    std::string path = ros::package::getPath("zivid_samples");
    path = path + "/config/only_2d_settings.yml";
    std::cout << "Got config from path: " << path << std::endl;
    zivid_camera::LoadSettingsFromFile file;
    file.request.file_path = path;
    if (load_settings_.call(file)) ROS_INFO("Loading image settings");
    else ROS_ERROR("Failed to call service load_settings_from_file");  
    capture();
  }
  else
  {
    flag_pub_ = false;
    flag_pub_info_ = false;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_img_calib");
  ros::NodeHandle n;

  ROS_INFO("Starting pub_img_calib.cpp");

  // CHECK(ros::service::waitForService("/zivid_camera/capture_2d", default_wait_duration));
  // CHECK(ros::service::waitForService("/zivid_camera/load_settings_from_file", default_wait_duration));

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  ros::Rate r_30HZ(30);

  auto image_color_sub = n.subscribe("/zivid_camera/color/image_color", 1, on_image_color);
  auto info_color_sub  = n.subscribe("/zivid_camera/color/camera_info", 1, on_info_color);

  ros::Publisher image_color_pub = n.advertise<sensor_msgs::Image>("/zivid/color/image_rect_color", 1);
  ros::Publisher info_color_pub = n.advertise<sensor_msgs::CameraInfo>("/zivid/color/camera_info", 1);

  ros::ServiceServer server_pub_img = n.advertiseService("/zivid_pub_img", &callback_zivid_pub_img);

  load_settings_ = n.serviceClient<zivid_camera::LoadSettingsFromFile>("/zivid_camera/load_settings_from_file");

  

  // ROS_INFO("Configuring image settings");
  // dynamic_reconfigure::Client<zivid_camera::Settings2DAcquisitionConfig> acquisition_0_client("/zivid_camera/"
  //                                                                                             "settings_2d/"
  //                                                                                             "acquisition_0/");

  // To initialize the cfg object we need to load the default configuration from the server.
  // The default values of settings depends on which Zivid camera model is connected.
  // zivid_camera::Settings2DAcquisitionConfig acquisition_0_config;
  // CHECK(acquisition_0_client.getDefaultConfiguration(acquisition_0_config, default_wait_duration));

  // acquisition_0_config.enabled = true;
  // acquisition_0_config.aperture = 5.66;
  // acquisition_0_config.exposure_time = 10000;
  // acquisition_0_config.brightness = 1.0;
  // CHECK(acquisition_0_client.setConfiguration(acquisition_0_config));

  // capture();

  while(ros::ok())
  {
    if(flag_pub_)image_color_pub.publish(*msg_img_);
    if(flag_pub_info_)info_color_pub.publish(msg_info_);
    
    ros::spinOnce();
    r_30HZ.sleep();
  }// end while()

  // ros::waitForShutdown();


  return 0;
}