////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_compliance_controller.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_COMPLIANCE_CONTROLLER_HPP_INCLUDED
#define CARTESIAN_COMPLIANCE_CONTROLLER_HPP_INCLUDED

// Project
#include <cartesian_compliance_controller/cartesian_compliance_controller.h>

// Other
#include <algorithm>
#include <map>

namespace cartesian_compliance_controller
{

template <class HardwareInterface>
CartesianComplianceController<HardwareInterface>::
CartesianComplianceController()
// Base constructor won't be called in diamond inheritance, so call that
// explicitly
: Base::CartesianControllerBase(),
  MotionBase::CartesianMotionController(),
  ForceBase::CartesianForceController(), m_hand_frame_control(true)
{
}

template <class HardwareInterface>
bool CartesianComplianceController<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  // Only one of them will call Base::init(hw,nh);
  MotionBase::init(hw,nh);
  ForceBase::init(hw,nh);

  // Connect dynamic reconfigure and overwrite the default values with values
  // on the parameter server. This is done automatically if parameters with
  // the according names exist.
  m_callback_type = std::bind(
      &CartesianComplianceController<HardwareInterface>::dynamicReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);

  m_dyn_conf_server.reset(
      new dynamic_reconfigure::Server<ComplianceConfig>(
        ros::NodeHandle(nh.getNamespace() + "/stiffness")));
  m_dyn_conf_server->setCallback(m_callback_type);

  return true;
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::
starting(const ros::Time& time)
{
  // Base::starting(time) will get called twice,
  // but that's fine.
  MotionBase::starting(time);
  ForceBase::starting(time);
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::
stopping(const ros::Time& time)
{
  MotionBase::stopping(time);
  ForceBase::stopping(time);
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes. This internal control needs some simulation time steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    ros::Duration internal_period(0.02);

    // Compute the net force
    ctrl::Vector6D error = computeComplianceError();
    
    if (m_hand_frame_control) {
      Base::computeJointControlCmds(Base::m_end_effector_link, error,internal_period);
    }
    else {
      Base::computeJointControlCmds(Base::m_robot_base_link, error,internal_period);
    }
     

    // Turn Cartesian error into joint motion
    
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();
}

template <class HardwareInterface>
ctrl::Vector6D CartesianComplianceController<HardwareInterface>::
computeComplianceError()
{
  ctrl::Vector6D motion_error, force_error, compliance_error;

  if (m_hand_frame_control) {
    motion_error = Base::displayInTipLink(MotionBase::computeMotionError(), Base::m_robot_base_link); // motion error in compliance reference
    force_error = ForceBase::m_ft_sensor_wrench + ForceBase::m_target_wrench;
    compliance_error = m_stiffness * motion_error + force_error;
  }
  else {
    motion_error = MotionBase::computeMotionError();
    force_error = Base::displayInBaseLink(ForceBase::m_ft_sensor_wrench,Base::m_end_effector_link) + ForceBase::m_target_wrench;
    compliance_error = m_stiffness * motion_error + force_error;
  }

  ctrl::Vector6D net_error;
  for (unsigned int i = 0; i < m_axes_control_type.size(); i++) {
    switch(m_axes_control_type.at(i)) {
      case 0:
        net_error[i] = 0;
        break;
      case 1:
        net_error[i] = motion_error[i];
        break;
      case 2:
        net_error[i] = force_error[i];
        break;
      case 3:
        net_error[i] = compliance_error[i];
        break;
    }
  }

  return net_error;
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::
dynamicReconfigureCallback(ComplianceConfig& config, uint32_t level)
{
  ctrl::Vector6D tmp;
  tmp[0] = config.trans_x;
  tmp[1] = config.trans_y;
  tmp[2] = config.trans_z;
  tmp[3] = config.rot_x;
  tmp[4] = config.rot_y;
  tmp[5] = config.rot_z;
  m_stiffness = tmp.asDiagonal();

  m_axes_control_type[0] = config.control_trans_x;
  m_axes_control_type[1] = config.control_trans_y;
  m_axes_control_type[2] = config.control_trans_z;
  m_axes_control_type[3] = config.control_rot_x;
  m_axes_control_type[4] = config.control_rot_y;
  m_axes_control_type[5] = config.control_rot_z;

  m_hand_frame_control = config.hand_frame_control;
}

} // namespace

#endif
