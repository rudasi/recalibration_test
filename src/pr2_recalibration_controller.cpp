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
  pr2_recalibration_values::values temp;

  for(int i = 0; i < num_elements; i++)
  {
     std::string my_transmission_name = robot_->model_->transmissions_[i]->name_;
     if(my_transmission_name.compare("r_gripper_motor") != 0)
     {
        int num_actuators = robot_->model_->transmissions_[i]->actuator_names_.size();
        ROS_INFO("number of actuators %d", num_actuators);
        ROS_INFO("transmission name is %s",my_transmission_name.c_str()); 

        std::vector<pr2_mechanism_model::JointState*> temp_joint_ptr;
        std::vector<pr2_hardware_interface::Actuator*> temp_actuator_ptr;
     
        for(int j = 0; j < num_actuators; j++)
        {
           std::string my_actuator_name = robot_->model_->transmissions_[i]->actuator_names_[j];
           ROS_INFO("Actuator name is %s",my_actuator_name.c_str());
           pr2_hardware_interface::Actuator* my_actuator = robot_->model_->getActuator(my_actuator_name);

	   pr2_mechanism_model::JointState* temp_joint = new pr2_mechanism_model::JointState;
	   pr2_hardware_interface::Actuator* temp_actuator = new pr2_hardware_interface::Actuator(*my_actuator);
	   temp_joint_ptr.push_back(temp_joint);

	   temp_actuator->state_.position_ = my_actuator->state_.zero_offset_;
	   temp_actuator_ptr.push_back(temp_actuator);


           temp.actuator_name.push_back(my_actuator_name); 
	
	   //temp_joint_ptr.erase(temp_joint_ptr.begin());
	   //temp_actuator_ptr.erase(temp_actuator_ptr.begin());

	   delete temp_joint;
	   delete temp_actuator;
        }

        robot_->model_->transmissions_[i]->propagatePosition(temp_actuator_ptr, temp_joint_ptr);
        for(int j = 0; j < num_actuators; j++)
        {
        //   temp.actuator_position.push_back(temp_actuator_ptr[j]->state_.position_);
        //   temp.actuator_offset.push_back(temp_actuator_ptr[j]->state_.zero_offset_); 
          temp.joint_position.push_back(temp_joint_ptr[j]->position_);
        }
        resp.array.push_back(temp); 
     }
  } 
  return true;
}

}

