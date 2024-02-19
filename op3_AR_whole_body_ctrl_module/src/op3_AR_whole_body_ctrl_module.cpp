#include "op3_AR_whole_body_ctrl_module/op3_AR_whole_body_ctrl_module.h"
#include <sstream>
#include <fstream>

using namespace adol;
using namespace robotis_op;

OP3ARWholeBodyCtrlModule::OP3ARWholeBodyCtrlModule()
    : control_cycle_msec_(8),
      control_cycle_sec_(0.008),
      DEBUG(false)
{
  enable_ = false;
  module_name_ = "AR_whole_body_ctrl_module";
  control_mode_ = robotis_framework::PositionControl;

  op3_kd_ = new OP3KinematicsDynamics(WholeBody);

  // result
  result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
  result_["r_sho_roll"] = new robotis_framework::DynamixelState();
  result_["r_el"] = new robotis_framework::DynamixelState();

  result_["l_sho_pitch"] = new robotis_framework::DynamixelState();
  result_["l_sho_roll"] = new robotis_framework::DynamixelState();
  result_["l_el"] = new robotis_framework::DynamixelState();

  result_["r_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["r_hip_roll"] = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["r_knee"] = new robotis_framework::DynamixelState();
  result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ank_roll"] = new robotis_framework::DynamixelState();

  result_["l_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["l_hip_roll"] = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["l_knee"] = new robotis_framework::DynamixelState();
  result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ank_roll"] = new robotis_framework::DynamixelState();

  result_["head_pan"] = new robotis_framework::DynamixelState();
  result_["head_tilt"] = new robotis_framework::DynamixelState();

  dyn_state_[0] = 0;
  dyn_state_[1] = result_["r_sho_pitch"];
  dyn_state_[2] = result_["l_sho_pitch"];
  dyn_state_[3] = result_["r_sho_roll"];
  dyn_state_[4] = result_["l_sho_roll"];
  dyn_state_[5] = result_["r_el"];
  dyn_state_[6] = result_["l_el"];
  dyn_state_[7] = result_["r_hip_yaw"];
  dyn_state_[8] = result_["l_hip_yaw"];
  dyn_state_[9] = result_["r_hip_roll"];
  dyn_state_[10] = result_["l_hip_roll"];
  dyn_state_[11] = result_["r_hip_pitch"];
  dyn_state_[12] = result_["l_hip_pitch"];
  dyn_state_[13] = result_["r_knee"];
  dyn_state_[14] = result_["l_knee"];
  dyn_state_[15] = result_["r_ank_pitch"];
  dyn_state_[16] = result_["l_ank_pitch"];
  dyn_state_[17] = result_["r_ank_roll"];
  dyn_state_[18] = result_["l_ank_roll"];
  dyn_state_[19] = result_["head_pan"];
  dyn_state_[20] = result_["head_tilt"];

  r_arm_angle_rad_[0] = 0.0;
  r_arm_angle_rad_[1] = 0.0;
  r_arm_angle_rad_[2] = 0.0;

  l_arm_angle_rad_[0] = 0.0;
  l_arm_angle_rad_[1] = 0.0;
  l_arm_angle_rad_[2] = 0.0;

  head_pan_angle_rad_ = 0.0;
  head_tilt_angle_rad_ = 0.0;

  r_leg_angle_rad_[0] = 0.0;
  r_leg_angle_rad_[1] = 0.0;
  r_leg_angle_rad_[2] = 0.0;
  r_leg_angle_rad_[3] = 0.0;
  r_leg_angle_rad_[4] = 0.0;
  r_leg_angle_rad_[5] = 0.0;

  l_leg_angle_rad_[0] = 0.0;
  l_leg_angle_rad_[1] = 0.0;
  l_leg_angle_rad_[2] = 0.0;
  l_leg_angle_rad_[3] = 0.0;
  l_leg_angle_rad_[4] = 0.0;
  l_leg_angle_rad_[5] = 0.0;

  body_height_m_ = 0.25065;

  com_state_[0] = 0;
  com_state_[1] = 0;
  curr_com_x_pos_m_ = prev_com_pos_x_ = 0;
  total_mass_kg_ = op3_kd_->calcTotalMass(0);
}

OP3ARWholeBodyCtrlModule::~OP3ARWholeBodyCtrlModule()
{
  queue_thread_.join();
  file.close();
}

void OP3ARWholeBodyCtrlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  client_ = new TCPClient();

  queue_thread_ = boost::thread(boost::bind(&OP3ARWholeBodyCtrlModule::queueThread, this));

  control_cycle_msec_ = control_cycle_msec;
  control_cycle_sec_ = control_cycle_msec_ * 0.001;

  for (std::map<std::string, robotis_framework::Dynamixel *>::iterator it = robot->dxls_.begin();
       it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel *dxl_info = it->second;

    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
  }

  std::time_t now = std::time(0);
  std::tm *now_t = std::localtime(&now);
  std::stringstream ss;
  ss << getenv("HOME") << "/Documents/" << now_t->tm_year + 1900;
  ss << std::setw(2) << std::setfill('0') << now_t->tm_mon + 1
     << std::setw(2) << std::setfill('0') << now_t->tm_mday
     << std::setw(2) << std::setfill('0') << now_t->tm_hour
     << std::setw(2) << std::setfill('0') << now_t->tm_min << ".txt";
  std::cout << ss.str() << std::endl;

  file.open(ss.str().c_str());
  if (file.is_open() == true)
    ROS_INFO("logging file is open");
  else
    ROS_ERROR("logging file is not open");
}

void OP3ARWholeBodyCtrlModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  ros_node.param<std::string>("server_ip_address", client_->server_ip_addr_, "127.0.0.1");

  if (client_->initialize(client_->server_ip_addr_, 12345))
    ROS_INFO("TCP Connected");
  else
    ROS_WARN("TCP Failed");
  /* publish topics */
  // status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);

  /* subscribe topics */
  // ros::Subscriber walking_param_sub = ros_node.subscribe("/robotis/walking/set_params", 0,
  //                                                        &WalkingModule::walkingParameterCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);

  int i = 0;
  while (ros_node.ok())
  {
    if (client_->receiveData() != -1)
    {
      angle_change_mutex_.lock();
      r_arm_angle_rad_[0] = client_->pose_msg_[0];
      r_arm_angle_rad_[1] = client_->pose_msg_[2];
      r_arm_angle_rad_[2] = client_->pose_msg_[4];

      l_arm_angle_rad_[0] = client_->pose_msg_[1];
      l_arm_angle_rad_[1] = client_->pose_msg_[3];
      l_arm_angle_rad_[2] = client_->pose_msg_[5];

      head_pan_angle_rad_ = client_->pose_msg_[6];
      head_tilt_angle_rad_ = client_->pose_msg_[7];

      body_height_m_ = client_->pose_msg_[8];
      angle_change_mutex_.unlock();
    }
    client_->com_state_[0] = com_state_[0];
    client_->com_state_[1] = com_state_[1];
    client_->sendData();
    callback_queue.callAvailable(duration);
  }
}

void OP3ARWholeBodyCtrlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                       std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  // angle_change_mutex_.lock();
  // result_["r_sho_pitch"]->goal_position_ = r_arm_angle_rad_[0];
  // result_["r_sho_roll"]->goal_position_ = r_arm_angle_rad_[1];
  // result_["r_el"]->goal_position_ = r_arm_angle_rad_[2];

  // result_["l_sho_pitch"]->goal_position_ = l_arm_angle_rad_[0];
  // result_["l_sho_roll"]->goal_position_ = l_arm_angle_rad_[1];
  // result_["l_el"]->goal_position_ = l_arm_angle_rad_[2];

  // result_["head_pan"]->goal_position_ = head_pan_angle_rad_;
  // result_["head_tilt"]->goal_position_ = head_tilt_angle_rad_;

  // op3_kd_->calcInverseKinematicsForRightLeg(r_leg_angle_rad_, 0, 0, -body_height_m_, 0, 0, 0);
  // op3_kd_->calcInverseKinematicsForLeftLeg(l_leg_angle_rad_, 0, 0, -body_height_m_, 0, 0, 0);
  // angle_change_mutex_.unlock();

  // result_["r_hip_yaw"]->goal_position_ = r_leg_angle_rad_[0];
  // result_["r_hip_roll"]->goal_position_ = r_leg_angle_rad_[1];
  // result_["r_hip_pitch"]->goal_position_ = r_leg_angle_rad_[2];
  // result_["r_knee"]->goal_position_ = r_leg_angle_rad_[3];
  // result_["r_ank_pitch"]->goal_position_ = r_leg_angle_rad_[4];
  // result_["r_ank_roll"]->goal_position_ = r_leg_angle_rad_[5];

  // result_["l_hip_yaw"]->goal_position_ = l_leg_angle_rad_[0];
  // result_["l_hip_roll"]->goal_position_ = l_leg_angle_rad_[1];
  // result_["l_hip_pitch"]->goal_position_ = l_leg_angle_rad_[2];
  // result_["l_knee"]->goal_position_ = l_leg_angle_rad_[3];
  // result_["l_ank_pitch"]->goal_position_ = l_leg_angle_rad_[4];
  // result_["l_ank_roll"]->goal_position_ = l_leg_angle_rad_[5];

  angle_change_mutex_.lock();
  dyn_state_[1]->goal_position_ = r_arm_angle_rad_[0];
  dyn_state_[3]->goal_position_ = r_arm_angle_rad_[1];
  dyn_state_[5]->goal_position_ = r_arm_angle_rad_[2];

  dyn_state_[2]->goal_position_ = l_arm_angle_rad_[0];
  dyn_state_[4]->goal_position_ = l_arm_angle_rad_[1];
  dyn_state_[6]->goal_position_ = l_arm_angle_rad_[2];

  dyn_state_[19]->goal_position_ = head_pan_angle_rad_;
  dyn_state_[20]->goal_position_ = head_tilt_angle_rad_;

  op3_kd_->calcInverseKinematicsForRightLeg(r_leg_angle_rad_, 0, 0, -body_height_m_, 0, 0, 0);
  op3_kd_->calcInverseKinematicsForLeftLeg(l_leg_angle_rad_, 0, 0, -body_height_m_, 0, 0, 0);
  angle_change_mutex_.unlock();

  dyn_state_[7]->goal_position_  = r_leg_angle_rad_[0];
  dyn_state_[9]->goal_position_  = r_leg_angle_rad_[1];
  dyn_state_[11]->goal_position_ = r_leg_angle_rad_[2];
  dyn_state_[13]->goal_position_ = r_leg_angle_rad_[3];
  dyn_state_[15]->goal_position_ = r_leg_angle_rad_[4];
  dyn_state_[17]->goal_position_ = r_leg_angle_rad_[5];

  dyn_state_[8]->goal_position_  = l_leg_angle_rad_[0];
  dyn_state_[10]->goal_position_ = l_leg_angle_rad_[1];
  dyn_state_[12]->goal_position_ = l_leg_angle_rad_[2];
  dyn_state_[14]->goal_position_ = l_leg_angle_rad_[3];
  dyn_state_[16]->goal_position_ = l_leg_angle_rad_[4];
  dyn_state_[18]->goal_position_ = l_leg_angle_rad_[5];

  op3_kd_->op3_link_data_[1]->joint_angle_ = dxls["r_sho_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[2]->joint_angle_ = dxls["l_sho_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[3]->joint_angle_ = dxls["r_sho_roll"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[4]->joint_angle_ = dxls["l_sho_roll"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[5]->joint_angle_ = dxls["r_el"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[6]->joint_angle_ = dxls["l_el"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[7]->joint_angle_ = dxls["r_hip_yaw"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[8]->joint_angle_ = dxls["l_hip_yaw"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[9]->joint_angle_ = dxls["r_hip_roll"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[10]->joint_angle_ = dxls["l_hip_roll"]->dxl_state_->present_position_ ;
  op3_kd_->op3_link_data_[11]->joint_angle_ = dxls["r_hip_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[12]->joint_angle_ = dxls["l_hip_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[13]->joint_angle_ = dxls["r_knee"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[14]->joint_angle_ = dxls["l_knee"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[15]->joint_angle_ = dxls["r_ank_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[16]->joint_angle_ = dxls["l_ank_pitch"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[17]->joint_angle_ = dxls["r_ank_roll"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[18]->joint_angle_ = dxls["l_ank_roll"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[19]->joint_angle_ = dxls["head_pan"]->dxl_state_->present_position_;
  op3_kd_->op3_link_data_[20]->joint_angle_ = dxls["head_tilt"]->dxl_state_->present_position_;
  
  op3_kd_->calcForwardKinematics(0);
  Eigen::MatrixXd com = op3_kd_->calcMC(0)/total_mass_kg_;

  curr_com_x_pos_m_ = com(0) - 0.5*(op3_kd_->op3_link_data_[30]->position_(0) + op3_kd_->op3_link_data_[30]->position_(0));
  com_state_[0] = curr_com_x_pos_m_;
  com_state_[1] = (curr_com_x_pos_m_ - prev_com_pos_x_)/control_cycle_sec_;
  prev_com_pos_x_ = curr_com_x_pos_m_;


  file << dyn_state_[1] ->goal_position_ << " " << dxls["r_sho_pitch"]->dxl_state_->present_position_ << " "
       << dyn_state_[2] ->goal_position_ << " " << dxls["l_sho_pitch"]->dxl_state_->present_position_ << " "
       << dyn_state_[3] ->goal_position_ << " " << dxls["r_sho_roll"]->dxl_state_->present_position_ << " "
       << dyn_state_[4] ->goal_position_ << " " << dxls["l_sho_roll"]->dxl_state_->present_position_ << " "
       << dyn_state_[5] ->goal_position_ << " " << dxls["r_el"]->dxl_state_->present_position_ << " "
       << dyn_state_[6] ->goal_position_ << " " << dxls["l_el"]->dxl_state_->present_position_ << " "
       << dyn_state_[7] ->goal_position_ << " " << dxls["r_hip_yaw"]->dxl_state_->present_position_ << " "
       << dyn_state_[8] ->goal_position_ << " " << dxls["l_hip_yaw"]->dxl_state_->present_position_ << " "
       << dyn_state_[9] ->goal_position_ << " " << dxls["r_hip_roll"]->dxl_state_->present_position_ << " "
       << dyn_state_[10]->goal_position_ << " " <<  dxls["l_hip_roll"]->dxl_state_->present_position_ << " "
       << dyn_state_[11]->goal_position_ << " " <<  dxls["r_hip_pitch"]->dxl_state_->present_position_ << " "
       << dyn_state_[12]->goal_position_ << " " <<  dxls["l_hip_pitch"]->dxl_state_->present_position_ << " "
       << dyn_state_[13]->goal_position_ << " " <<  dxls["r_knee"]->dxl_state_->present_position_ << " "
       << dyn_state_[14]->goal_position_ << " " <<  dxls["l_knee"]->dxl_state_->present_position_ << " "
       << dyn_state_[15]->goal_position_ << " " <<  dxls["r_ank_pitch"]->dxl_state_->present_position_ << " "
       << dyn_state_[16]->goal_position_ << " " <<  dxls["l_ank_pitch"]->dxl_state_->present_position_ << " "
       << dyn_state_[17]->goal_position_ << " " <<  dxls["r_ank_roll"]->dxl_state_->present_position_ << " "
       << dyn_state_[18]->goal_position_ << " " <<  dxls["l_ank_roll"]->dxl_state_->present_position_ << " "
       << dyn_state_[19]->goal_position_ << " " <<  dxls["head_pan"]->dxl_state_->present_position_ << " "
       << dyn_state_[20]->goal_position_ << " " <<  dxls["head_tilt"]->dxl_state_->present_position_ << " "
       << com_state_[0] << " " << com_state_[1]
       << std::endl;

  // file << result_["r_sho_pitch"]->goal_position_ << result_["r_sho_pitch"]->present_position_
  //      << result_["r_sho_roll"]->goal_position_ << result_["r_sho_roll"]->present_position_
  //      << result_["r_el"]->goal_position_ << result_["r_el"]->present_position_
  //      << result_["l_sho_pitch"]->goal_position_ << result_["l_sho_pitch"]->present_position_
  //      << result_["l_sho_roll"]->goal_position_ << result_["l_sho_roll"]->present_position_
  //      << result_["l_el"]->goal_position_ << result_["l_el"]->present_position_
  //      << result_["r_hip_yaw"]->goal_position_ << result_["r_hip_yaw"]->present_position_
  //      << result_["r_hip_roll"]->goal_position_ << result_["r_hip_roll"]->present_position_
  //      << result_["r_hip_pitch"]->goal_position_ << result_["r_hip_pitch"]->present_position_
  //      << result_["r_knee"]->goal_position_ << result_["r_knee"]->present_position_
  //      << result_["r_ank_pitch"]->goal_position_ << result_["r_ank_pitch"]->present_position_
  //      << result_["r_ank_roll"]->goal_position_ << result_["r_ank_roll"]->present_position_
  //      << result_["l_hip_yaw"]->goal_position_ << result_["l_hip_yaw"]->present_position_
  //      << result_["l_hip_roll"]->goal_position_ << result_["l_hip_roll"]->present_position_
  //      << result_["l_hip_pitch"]->goal_position_ << result_["l_hip_pitch"]->present_position_
  //      << result_["l_knee"]->goal_position_ << result_["l_knee"]->present_position_
  //      << result_["l_ank_pitch"]->goal_position_ << result_["l_ank_pitch"]->present_position_
  //      << result_["l_ank_roll"]->goal_position_ << result_["l_ank_roll"]->present_position_
  //      << result_["head_pan"]->goal_position_ << result_["head_pan"]->present_position_
  //      << result_["head_tilt"]->goal_position_ << result_["head_tilt"]->present_position_
  //      << curr_com_x_pos_m_
  //      << std::endl;
}

void OP3ARWholeBodyCtrlModule::onModuleEnable()
{
  ROS_INFO("AR Whole-Body Ctrl Module is enabled");
}

void OP3ARWholeBodyCtrlModule::onModuleDisable()
{
  ROS_INFO("AR Whole-Body Ctrl Module is enabled");
}

void OP3ARWholeBodyCtrlModule::stop()
{
}

bool OP3ARWholeBodyCtrlModule::isRunning()
{
  return false;
}