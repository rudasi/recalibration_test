#include "pr2_recalibration_values/pr2_recalibration_controller.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include "pr2_recalibration_values/values.h"

PLUGINLIB_DECLARE_CLASS(pr2_recalibration_values, Pr2RecalibrationPlugin, controller::Pr2RecalibrationValues, pr2_controller_interface::Controller)

namespace controller
{

Pr2RecalibrationValues::Pr2RecalibrationValues()
: joint_state_(NULL), robot_(NULL)
{

}

bool Pr2RecalibrationValues::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;
  robot_ = robot;

  //need to advertise service and create srv file
  srv_ = n.advertiseService("get_calibration_offset", &Pr2RecalibrationValues::getOffset, this);
  return true;
}
   
void Pr2RecalibrationValues::update()
{
}

void Pr2RecalibrationValues::stopping()
{
}

void Pr2RecalibrationValues::starting()
{
}

bool Pr2RecalibrationValues::getOffset(pr2_recalibration_values::GetCalibrationOffset::Request& req, pr2_recalibration_values::GetCalibrationOffset::Response& resp)
{
  if (robot_ == NULL)
  {
    ROS_ERROR("robot is null");
    return false;
  }
  if (robot_->model_ == NULL)
  {
    ROS_ERROR("robot_->model_ is null");
    return false;
  }

  int num_transmissions = robot_->model_->transmissions_.size();
  ROS_INFO("number of transmissions %d",num_transmissions);

  for(int i = 0; i < num_transmissions; i++)
  {
    pr2_mechanism_model::Transmission *transmission = robot_->model_->transmissions_[i];

    pr2_recalibration_values::values temp_values;
    std::string my_transmission_name = transmission->name_;
    temp_values.transmission_name = my_transmission_name;

    ROS_INFO("Transmission name is %s",my_transmission_name.c_str());

    bool ok = true;

    int num_actuators = transmission->actuator_names_.size();
    ROS_INFO(" Number of actuators %d", num_actuators);
    std::vector<pr2_hardware_interface::Actuator*> temp_actuator_list;
    for(int j = 0; j < num_actuators; j++)
    {
      std::string my_actuator_name = transmission->actuator_names_[j];
      ROS_INFO(" Actuator name is %s",my_actuator_name.c_str());
      pr2_hardware_interface::Actuator* my_actuator = robot_->model_->getActuator(my_actuator_name);
      if(my_actuator == NULL)
      {
	ROS_ERROR("Could not find actuator: %s", my_actuator_name.c_str());
	ok = false;
	break;
      }
      else
      {
	pr2_hardware_interface::Actuator* temp_actuator = new pr2_hardware_interface::Actuator(*my_actuator);
	temp_actuator->state_.position_ = my_actuator->state_.zero_offset_;
	temp_actuator_list.push_back(temp_actuator);
	temp_values.actuator_name.push_back(my_actuator_name);
      }
    }

    int num_joints = transmission->joint_names_.size();
    ROS_INFO(" Number of joints %d", num_joints);
    std::vector<pr2_mechanism_model::JointState*> temp_joint_list;
    for(int j = 0; j < num_joints; j++)
    {
      std::string my_joint_name = transmission->joint_names_[j];
      ROS_INFO(" Joint name is %s",my_joint_name.c_str());
      pr2_mechanism_model::JointState* my_joint = robot_->getJointState(my_joint_name);
      if(my_joint == NULL)
      {
	ROS_ERROR("Could not find joint: %s", my_joint_name.c_str());
	ok = false;
	break;
      }
      else
      {
	pr2_mechanism_model::JointState* temp_joint = new pr2_mechanism_model::JointState(*my_joint);
	temp_joint_list.push_back(temp_joint);
	temp_values.joint_name.push_back(my_joint_name);
      }
    }

    if (ok)
    {
      transmission->propagatePosition(temp_actuator_list, temp_joint_list);
      for(int j = 0; j < num_joints; j++)
      {
	temp_values.joint_position.push_back(temp_joint_list[j]->position_);
      }
      for(int j = 0; j < num_actuators; j++)
      {
	temp_values.actuator_position.push_back(temp_actuator_list[j]->state_.position_);
	temp_values.actuator_offset.push_back(temp_actuator_list[j]->state_.zero_offset_);
      }
      resp.array.push_back(temp_values);
    }

    for(int j = 0; j < num_actuators; j++)
    {
      delete temp_actuator_list[j];
    }
    for(int j = 0; j < num_joints; j++)
    {
      delete temp_joint_list[j];
    }

  } // end foreach transmission

  ROS_INFO("SUCCESSSS!!!!");

  return true;
}



} //end namespace controller


