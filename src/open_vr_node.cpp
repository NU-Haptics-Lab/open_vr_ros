#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <iostream>
#include <format>
#include "vr_interface.h"

// messages
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>

// services
#include <std_srvs/srv/empty.hpp>
#include <open_vr_ros/srv/set_origin.hpp>



#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;



void mySigintHandler(int sig){
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  // All the default sigint handler does is call shutdown()
  rclcpp::shutdown();
}

// import from opengl sample
std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL )
{
  uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
  if( unRequiredBufferLen == 0 )
    return "";

  char *pchBuffer = new char[ unRequiredBufferLen ];
  unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
  std::string sResult = pchBuffer;
  delete [] pchBuffer;
  return sResult;
}

class OPEN_VRnode
{
  public:
    OPEN_VRnode(int rate);
    bool Init();
    void Run();
    bool DeviceOn(int index);
    void Status();
    void Shutdown();
    bool setOriginCB(const std::shared_ptr<open_vr_ros::srv::SetOrigin::Request> req, std::shared_ptr<open_vr_ros::srv::SetOrigin::Response> res);
    bool setOriginCB_empty(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    std::string GetDeviceString(int index);
    rclcpp::Node::SharedPtr nh_ptr_;
    VRInterface vr_;
    double tf_matrix_[3][4];
    

  private:
    rclcpp::Rate loop_rate_;
    std::vector<double> offset_;
    double offset_yaw_;
    std::string frame_prefix_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // subs

    // pubs
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr hmd_tfs;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist0_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist2_pub_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr> button_states_pubs_map;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_HMD_on;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_LEFT_on;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_RIGHT_on;

    // services
    rclcpp::Service<open_vr_ros::srv::SetOrigin>::SharedPtr set_origin_server_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_origin_server_empty_;

    // timers
    rclcpp::TimerBase::SharedPtr timer_run;
    rclcpp::TimerBase::SharedPtr timer_status;
};

/// @brief constructor, ctor
/// @param rate 
OPEN_VRnode::OPEN_VRnode(int rate)
  : loop_rate_(rate)
  , vr_()
  , offset_({0, 0, 0})
  , offset_yaw_(0)
{
  // make the ros2 node
  nh_ptr_ = rclcpp::Node::make_shared("open_vr_node");

  // tf classes
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_ptr_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh_ptr_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // declare parameters
  nh_ptr_->declare_parameter("offset", offset_); // default values
  nh_ptr_->declare_parameter("yaw", offset_yaw_); // default values
  nh_ptr_->declare_parameter("FRAME_PREFIX", ""); // default values

  // get parameters
  offset_ = nh_ptr_->get_parameter("offset").as_double_array();
  offset_yaw_ = nh_ptr_->get_parameter("yaw").as_double();
  frame_prefix_ = nh_ptr_->get_parameter("FRAME_PREFIX").as_string();

  // rest of constructor
  RCLCPP_INFO(nh_ptr_->get_logger(), " [OPEN_VR] Offset offset: [%2.3f , %2.3f, %2.3f] %2.3f", offset_[0], offset_[1], offset_[2], 
  offset_yaw_);

  // subs

  // pubs
  // make the qos_profile
  rclcpp::QoS qos_profile(10);
  qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  qos_profile.liveliness_lease_duration(1s); 
  
  hmd_tfs = nh_ptr_->create_publisher<geometry_msgs::msg::TransformStamped>("open_vr/hmd", qos_profile);
  twist0_pub_ = nh_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>("open_vr/twist0", qos_profile);
  twist1_pub_ = nh_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>("open_vr/twist1", qos_profile);
  twist2_pub_ = nh_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>("open_vr/twist2", qos_profile);
  pub_status_HMD_on = nh_ptr_->create_publisher<std_msgs::msg::Bool>("open_vr/HMD/on", qos_profile);
  pub_status_LEFT_on = nh_ptr_->create_publisher<std_msgs::msg::Bool>("open_vr/LEFT/on", qos_profile);
  pub_status_RIGHT_on = nh_ptr_->create_publisher<std_msgs::msg::Bool>("open_vr/RIGHT/on", qos_profile);

  // services
  set_origin_server_ = nh_ptr_->create_service<open_vr_ros::srv::SetOrigin>("open_vr/set_origin", std::bind(&OPEN_VRnode::setOriginCB, this, _1, _2));
  set_origin_server_empty_ = nh_ptr_->create_service<std_srvs::srv::Empty>("open_vr/set_origin_empty", std::bind(&OPEN_VRnode::setOriginCB_empty, this, _1, _2));

  // timers
  timer_run = nh_ptr_->create_wall_timer(loop_rate_.period(), std::bind(&OPEN_VRnode::Run, this));
  timer_status = nh_ptr_->create_wall_timer(1s, std::bind(&OPEN_VRnode::Status, this)); // once a second


}

bool OPEN_VRnode::Init()
{
  //  Set logging functions
  
  // vr_.setDebugMsgCallback(handleDebugMessages);
  // vr_.setInfoMsgCallback(handleInfoMessages);
  // vr_.setErrorMsgCallback(handleErrorMessages);

  if (!vr_.Init())
  {
    return false;
  }

  return true;
}

void OPEN_VRnode::Shutdown()
{
  vr_.Shutdown();
}

/// @brief Service callback for setting the origin
/// @param req 
/// @param res 
/// @return 
bool OPEN_VRnode::setOriginCB(const std::shared_ptr<open_vr_ros::srv::SetOrigin::Request> req, std::shared_ptr<open_vr_ros::srv::SetOrigin::Response> res)
{
  try{
    RCLCPP_INFO(nh_ptr_->get_logger(), "SetOriginCB Service Initiated");
    // extract from message
    offset_[0] = req->x;
    offset_[1] = req->y;
    offset_[2] = req->z;
    offset_yaw_ = req->yaw;

    nh_ptr_->set_parameter(rclcpp::Parameter("open_vr/offset", offset_));
    nh_ptr_->set_parameter(rclcpp::Parameter("open_vr/yaw", offset_yaw_));
    RCLCPP_INFO(nh_ptr_->get_logger(), " [OPEN_VR] New offset offset: [%2.3f , %2.3f, %2.3f] %2.3f", offset_[0], offset_[1], offset_[2], offset_yaw_);

    res->success = true;
  } catch(...) {
    res->success = false;
  }
  return res->success;
}

/// @brief Service callback for setting the origin to the HMD
/// @param req 
/// @param res 
/// @return 
bool OPEN_VRnode::setOriginCB_empty(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  double tf_matrix[3][4];
  int index = 0, dev_type; // index = 0 should be the HMD
  // It's a HMD
  // +y is up
  // +x is to the right
  // -z is going away from you
  dev_type = vr_.GetDeviceMatrix(0, tf_matrix);

  tf2::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                           tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                           tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);
  tf2::Vector3 c_z;
  c_z = rot_matrix*tf2::Vector3(0,0,1);
  c_z[1] = 0;
  c_z.normalize();
  double new_yaw = acos(tf2::Vector3(0,0,1).dot(c_z)) + M_PI_2;
  if (c_z[0] < 0) new_yaw = -new_yaw;
  offset_yaw_ = -new_yaw;

  tf2::Vector3 new_offset;
  tf2::Matrix3x3 new_rot;
  new_rot.setRPY(0, 0, offset_yaw_);
  new_offset = new_rot*tf2::Vector3(-tf_matrix[0][3], tf_matrix[2][3], -tf_matrix[1][3]);

  auto req2 = std::make_shared<open_vr_ros::srv::SetOrigin::Request>();
  auto res2 = std::make_shared<open_vr_ros::srv::SetOrigin::Response>();
  req2->x = new_offset[0];
  req2->y = new_offset[1];
  req2->z = new_offset[2];
  req2->yaw = -new_yaw;
  
  return setOriginCB(req2, res2);
}

std::string OPEN_VRnode::GetDeviceString(int index)
{
    std::string cur_sn = GetTrackedDeviceString(vr_.pHMD_, index, vr::Prop_SerialNumber_String);
    std::replace(cur_sn.begin(), cur_sn.end(), '-', '_');
    return cur_sn;
}

/// @brief timer run callback
void OPEN_VRnode::Run()
{
  // update the openvr state
  vr_.Update();

  for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
  {
    int dev_type = vr_.GetDeviceMatrix(i, tf_matrix_);

    // No device
    if (dev_type == 0)
      continue;

    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(tf_matrix_[0][3], tf_matrix_[1][3], tf_matrix_[2][3]));

    tf2::Quaternion quat;
    tf2::Matrix3x3 rot_matrix(tf_matrix_[0][0], tf_matrix_[0][1], tf_matrix_[0][2],
                              tf_matrix_[1][0], tf_matrix_[1][1], tf_matrix_[1][2],
                              tf_matrix_[2][0], tf_matrix_[2][1], tf_matrix_[2][2]);

    rot_matrix.getRotation(quat);
    tf.setRotation(quat);
    // get device serial number
    auto cur_sn = GetDeviceString(i);

    // It's a HMD
    if (dev_type == 1)
    {
      geometry_msgs::msg::Transform msg_tf = tf2::toMsg(tf);
      geometry_msgs::msg::TransformStamped tfs;
      tfs.transform = msg_tf;
      tfs.header.stamp = nh_ptr_->get_clock()->now();
      tfs.header.frame_id = frame_prefix_ + "open_vr";
      tfs.child_frame_id = frame_prefix_ + "hmd";
      tf_broadcaster_->sendTransform(tfs);
      hmd_tfs->publish(tfs); // also publish directly to a topic
    }
    // It's a controller
    if (dev_type == 2)
    {
      geometry_msgs::msg::Transform msg_tf = tf2::toMsg(tf);
      geometry_msgs::msg::TransformStamped tfs;
      tfs.transform = msg_tf;
      tfs.header.stamp = nh_ptr_->get_clock()->now();
      tfs.header.frame_id = frame_prefix_ + "open_vr";
      tfs.child_frame_id = frame_prefix_ + "controller_" + cur_sn;
      tf_broadcaster_->sendTransform(tfs);

      vr::VRControllerState_t state;
      vr_.HandleInput(i, state);
      sensor_msgs::msg::Joy joy;
      joy.header.stamp = nh_ptr_->get_clock()->now();
      joy.header.frame_id = frame_prefix_ + "controller_" + cur_sn;
      joy.buttons.assign(BUTTON_NUM, 0);
      joy.axes.assign(AXES_NUM, 0.0); // x-axis, y-axis
      if ((1LL << vr::k_EButton_ApplicationMenu) & state.ulButtonPressed)
        joy.buttons[0] = 1;
      if ((1LL << vr::k_EButton_SteamVR_Trigger) & state.ulButtonPressed)
        joy.buttons[1] = 1;
      if ((1LL << vr::k_EButton_SteamVR_Touchpad) & state.ulButtonPressed)
        joy.buttons[2] = 1;
      if ((1LL << vr::k_EButton_Grip) & state.ulButtonPressed)
        joy.buttons[3] = 1;
      // TrackPad's axis
      joy.axes[0] = state.rAxis[0].x;
      joy.axes[1] = state.rAxis[0].y;
      // Trigger's axis
      joy.axes[2] = state.rAxis[1].x;
      //        #include <bitset> // bit debug
      //        std::cout << static_cast<std::bitset<64> >(state.ulButtonPressed) << std::endl;
      //        std::cout << static_cast<std::bitset<64> >(state.ulButtonTouched) << std::endl;
      if (button_states_pubs_map.count(cur_sn) == 0)
      {
        button_states_pubs_map[cur_sn] = nh_ptr_->create_publisher<sensor_msgs::msg::Joy>("open_vr/controller_" + cur_sn + "/joy", 10);
      }
      button_states_pubs_map[cur_sn]->publish(joy);
    }
    // It's a tracker
    if (dev_type == 3)
    {
      geometry_msgs::msg::Transform msg_tf = tf2::toMsg(tf);
      geometry_msgs::msg::TransformStamped tfs;
      tfs.transform = msg_tf;
      tfs.header.stamp = nh_ptr_->get_clock()->now();
      tfs.header.frame_id = frame_prefix_ + "open_vr";
      tfs.child_frame_id = frame_prefix_ + "tracker_" + cur_sn;
      tf_broadcaster_->sendTransform(tfs);
    }
    // It's a lighthouse
    if (dev_type == 4)
    {
      geometry_msgs::msg::Transform msg_tf = tf2::toMsg(tf);
      geometry_msgs::msg::TransformStamped tfs;
      tfs.transform = msg_tf;
      tfs.header.stamp = nh_ptr_->get_clock()->now();
      tfs.header.frame_id = frame_prefix_ + "open_vr";
      tfs.child_frame_id = frame_prefix_ + "lighthouse_" + cur_sn;
      tf_broadcaster_->sendTransform(tfs);
    }
  }

  // Publish corrective transform
  tf2::Transform tf_offset;
  tf_offset.setOrigin(tf2::Vector3(offset_[0], offset_[1], offset_[2]));
  tf2::Quaternion quat_offset;
  quat_offset.setRPY(M_PI / 2, 0, offset_yaw_);
  tf_offset.setRotation(quat_offset);

  geometry_msgs::msg::Transform msg_tf = tf2::toMsg(tf_offset);
  geometry_msgs::msg::TransformStamped tfs;
  tfs.transform = msg_tf;
  tfs.header.stamp = nh_ptr_->get_clock()->now();
  tfs.header.frame_id = frame_prefix_ + "open_vr_calibrated";
  tfs.child_frame_id = frame_prefix_ + "open_vr";
  tf_broadcaster_->sendTransform(tfs);

  // Publish twist messages for controller1 and controller2
  double lin_vel[3], ang_vel[3];
  if (vr_.GetDeviceVel(0, lin_vel, ang_vel))
  {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = lin_vel[0];
    twist_msg.linear.y = lin_vel[1];
    twist_msg.linear.z = lin_vel[2];
    twist_msg.angular.x = ang_vel[0];
    twist_msg.angular.y = ang_vel[1];
    twist_msg.angular.z = ang_vel[2];

    geometry_msgs::msg::TwistStamped twist_msg_stamped;
    twist_msg_stamped.header.stamp = nh_ptr_->get_clock()->now();
    twist_msg_stamped.header.frame_id = frame_prefix_ + "open_vr";
    twist_msg_stamped.twist = twist_msg;

    twist0_pub_->publish(twist_msg_stamped);

    // std::cout<<"HMD:";
    // std::cout<<twist_msg_stamped;
  }
  if (vr_.GetDeviceVel(1, lin_vel, ang_vel))
  {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = lin_vel[0];
    twist_msg.linear.y = lin_vel[1];
    twist_msg.linear.z = lin_vel[2];
    twist_msg.angular.x = ang_vel[0];
    twist_msg.angular.y = ang_vel[1];
    twist_msg.angular.z = ang_vel[2];

    geometry_msgs::msg::TwistStamped twist_msg_stamped;
    twist_msg_stamped.header.stamp = nh_ptr_->get_clock()->now();
    twist_msg_stamped.header.frame_id = frame_prefix_ + "open_vr";
    twist_msg_stamped.twist = twist_msg;

    twist1_pub_->publish(twist_msg_stamped);

    // std::cout<<"Controller 1:";
    // std::cout<<twist_msg_stamped;
  }
  if (vr_.GetDeviceVel(2, lin_vel, ang_vel))
  {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = lin_vel[0];
    twist_msg.linear.y = lin_vel[1];
    twist_msg.linear.z = lin_vel[2];
    twist_msg.angular.x = ang_vel[0];
    twist_msg.angular.y = ang_vel[1];
    twist_msg.angular.z = ang_vel[2];

    geometry_msgs::msg::TwistStamped twist_msg_stamped;
    twist_msg_stamped.header.stamp = nh_ptr_->get_clock()->now();
    twist_msg_stamped.header.frame_id = frame_prefix_ + "open_vr";
    twist_msg_stamped.twist = twist_msg;

    twist2_pub_->publish(twist_msg_stamped);

    // std::cout<<"Controller 2:";
    // std::cout<<twist_msg_stamped;
  }
}

bool OPEN_VRnode::DeviceOn(int index)
{
  return vr_.device_poses_[index].bDeviceIsConnected && vr_.device_poses_[index].eTrackingResult == vr::TrackingResult_Running_OK;
}

void OPEN_VRnode::Status()
{
  // bool msg
  std_msgs::msg::Bool bool_msg;
  bool hmd = false;
  bool left = false;
  bool right = false;

  // iterate through connected devices
  for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
  {
    auto type = vr_.pHMD_->GetTrackedDeviceClass(i);

    // HMD
    if (type == vr::TrackedDeviceClass::TrackedDeviceClass_HMD)
    {
      hmd = DeviceOn(i);
    }

    // Tracker
    if (type == vr::TrackedDeviceClass::TrackedDeviceClass_GenericTracker)
    {
      auto name = GetDeviceString(i);
      std::cout << "status name: " << name << std::endl;

      if (name.find("B7233729") != std::string::npos)
      {
        left = DeviceOn(i);
      }

      if (name.find("748927CF") != std::string::npos)
      {
        right = DeviceOn(i);
      }
    }
  }

  // publish the statusses
  bool_msg.data = hmd;
  pub_status_HMD_on->publish(bool_msg);

  bool_msg.data = left;
  pub_status_LEFT_on->publish(bool_msg);

  bool_msg.data = right;
  pub_status_RIGHT_on->publish(bool_msg);
}

// Main
int main(int argc, char** argv){
  signal(SIGINT, mySigintHandler);
  rclcpp::init(argc, argv);
  std::cout << "RCLCPP Init'd\n";

  OPEN_VRnode nodeApp(60);
  std::cout << "Made Open_VR node class\n";
  bool init = nodeApp.Init();
  std::cout << "nodeApp: init'd\n";

  if (!init){
    nodeApp.Shutdown();
    return 1;
  }
  
  rclcpp::spin(nodeApp.nh_ptr_);
  nodeApp.Shutdown();
  

  return 0;
};
