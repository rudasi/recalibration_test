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
  int num_elements = robot_->model_->transmissions_.size();
  ROS_INFO("number of transmissions %d",num_elements);

  for(int i = 0; i < num_elements; i++)
  {
     int num_actuators = robot_->model_->transmissions_[i]->actuator_names_.size();
     ROS_INFO("number of actuators %d", num_actuators);
     //std::string my_transmission_name = robot_->model_->transmissions_[0]->name_;
     //ROS_INFO("Transmission name is %s",my_transmission_name.c_str());
     //resp.array.push_back(robot_->model_->transmissions_[i]->name_); 
     pr2_recalibration_values::values temp;
     for(int j = 0; j < num_actuators; j++)
     {
        std::string my_actuator_name = robot_->model_->transmissions_[i]->actuator_names_[j];
        ROS_INFO("Actuator name is %s",my_actuator_name.c_str());
        pr2_hardware_interface::Actuator* my_actuator = robot_->model_->getActuator(my_actuator_name);
        temp.actuator_name.push_back(my_actuator_name); 
        temp.position.push_back(my_actuator->state_.position_);
        temp.offset.push_back(my_actuator->state_.zero_offset_); 
     }
     resp.array.push_back(temp); 
  } 
  return true;
}

}

