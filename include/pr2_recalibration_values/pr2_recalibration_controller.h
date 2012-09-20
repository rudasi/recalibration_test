#ifndef PR2_RECALIBRATION_CONTROLLER_H
#define PR2_RECALIBRATION_CONTROLLER_H

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_recalibration_values/GetCalibrationOffset.h>

namespace controller
{

class Pr2RecalibrationValues: public pr2_controller_interface::Controller
{
public:
  Pr2RecalibrationValues();
 // virtual ~Pr2RecalibrationValues();
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
  bool getOffset(pr2_recalibration_values::GetCalibrationOffset::Request& req, pr2_recalibration_values::GetCalibrationOffset::Response& resp);

private: 
  pr2_mechanism_model::JointState* joint_state_; 
  pr2_mechanism_model::RobotState* robot_;
  ros::NodeHandle node_;
  ros::ServiceServer srv_;
};
}

#endif
