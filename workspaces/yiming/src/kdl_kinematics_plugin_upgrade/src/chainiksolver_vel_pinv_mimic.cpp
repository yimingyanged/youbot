// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

// Modified to account for "mimic" joints, i.e. joints whose motion has a
// linear relationship to that of another joint.
// Copyright  (C)  2013  Sachin Chitta, Willow Garage

#include <moveit/kdl_kinematics_plugin_upgrade/chainiksolver_vel_pinv_mimic.hpp>
#include <ros/console.h>
#include <Eigen/Eigen>

namespace KDL
{
ChainIkSolverVel_pinv_mimic_upgrade::ChainIkSolverVel_pinv_mimic_upgrade(const Chain& _chain, const Eigen::MatrixXd& Winv_, const JntArray& q_comfortable_,
    int _num_mimic_joints, int _num_redundant_joints,
    double _eps, int _maxiter, int dim_,
    kdl_kinematics_plugin::IKType ik_type_, double Kp_,
    double lambda_, bool use_null_, bool use_weighting_):
  chain(_chain),
  jnt2jac(chain),
  jac(chain.getNrOfJoints()),
  jac_locked(chain.getNrOfJoints()-_num_redundant_joints-_num_mimic_joints),
  jac_reduced(chain.getNrOfJoints()-_num_mimic_joints),
  pinvJ(chain.getNrOfJoints()-_num_mimic_joints,dim_),
  lambda(lambda_),
  Winv(Eigen::MatrixXd::Identity(chain.getNrOfJoints()-_num_mimic_joints,chain.getNrOfJoints()-_num_mimic_joints)),
  C(Eigen::MatrixXd::Identity(dim_,dim_)*lambda_),
  Winv_locked(Eigen::MatrixXd::Identity(chain.getNrOfJoints()-_num_redundant_joints-_num_mimic_joints,chain.getNrOfJoints()-_num_redundant_joints-_num_mimic_joints)),
  eps(_eps),
  maxiter(_maxiter),
  qdot_out_reduced(chain.getNrOfJoints()-_num_mimic_joints),
  num_mimic_joints(_num_mimic_joints),
  num_redundant_joints(_num_redundant_joints),
  redundant_joints_locked(false),
  use_nullspace(use_null_),
  q_comfortable(chain.getNrOfJoints()),
  q_comfortable_reduced(chain.getNrOfJoints()-_num_mimic_joints),
  q_comfortable_locked(chain.getNrOfJoints()-_num_mimic_joints-_num_redundant_joints),
  q(chain.getNrOfJoints()-_num_mimic_joints),
  Kp(Kp_),
  ik_type(ik_type_),
  xd(dim_),
  task_dim(dim_)
{
  mimic_joints_.resize(chain.getNrOfJoints());
  for(std::size_t i=0; i < mimic_joints_.size(); ++i)
    mimic_joints_[i].reset(i);

  if(use_weighting_)
  {
    for(int i=0;i<Winv_.rows();i++) Winv(i,i)=Winv_(i,i);
  }
  if(use_null_)
    for(int i=0;i<q_comfortable.rows();i++) q_comfortable(i)=q_comfortable_(i);
  //for(int i=Winv.rows()-2;i>=0;i--) Winv(i,i)=Winv(i+1,i+1)*0.3;
}

ChainIkSolverVel_pinv_mimic_upgrade::~ChainIkSolverVel_pinv_mimic_upgrade()
{
}

bool ChainIkSolverVel_pinv_mimic_upgrade::setMimicJoints(const std::vector<kdl_kinematics_plugin::JointMimic> & mimic_joints)
{
  if(mimic_joints.size() != chain.getNrOfJoints())
    return false;

  for(std::size_t i=0; i < mimic_joints.size(); ++i)
  {
    if(mimic_joints[i].map_index >= chain.getNrOfJoints())
      return false;
  }
  mimic_joints_ = mimic_joints;
  return true;
}

bool ChainIkSolverVel_pinv_mimic_upgrade::setRedundantJointsMapIndex(const std::vector<unsigned int> & redundant_joints_map_index)
{
  if(redundant_joints_map_index.size() != chain.getNrOfJoints()-num_mimic_joints-num_redundant_joints)
    return false;

  for(std::size_t i=0; i < redundant_joints_map_index.size(); ++i)
  {
    if(redundant_joints_map_index[i] >= chain.getNrOfJoints()-num_mimic_joints)
      return false;
  }
  locked_joints_map_index = redundant_joints_map_index;
  return true;
}

bool ChainIkSolverVel_pinv_mimic_upgrade::jacToJacReduced(const Jacobian &jac, Jacobian &jac_reduced_l)
{
  jac_reduced_l.data.setZero();
  for(std::size_t i=0; i < chain.getNrOfJoints(); ++i)
  {
    Twist vel1 = jac_reduced_l.getColumn(mimic_joints_[i].map_index);
    Twist vel2 = jac.getColumn(i);
    Twist result = vel1 + (mimic_joints_[i].multiplier*vel2);
    jac_reduced_l.setColumn(mimic_joints_[i].map_index,result);
  }
  return true;
}

bool ChainIkSolverVel_pinv_mimic_upgrade::jacToJacLocked(const Jacobian &jac, Jacobian &jac_locked)
{
  jac_locked.data.setZero();
  for(std::size_t i=0; i < chain.getNrOfJoints()-num_mimic_joints-num_redundant_joints; ++i)
  {
    jac_locked.setColumn(i, jac.getColumn(locked_joints_map_index[i]));
  }
  return true;
}

int ChainIkSolverVel_pinv_mimic_upgrade::CartToJntRedundant(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
  Eigen::MatrixXd J;
  qdot_out.data.setZero();
  //Let the ChainJntToJacSolver calculate the jacobian "jac" for
  //the current joint positions "q_in". This will include the mimic joints
  q.resize(chain.getNrOfJoints()-num_mimic_joints-num_redundant_joints);
  if(num_mimic_joints > 0)
  {
    jnt2jac.JntToJac(q_in,jac);
    //Now compute the actual jacobian that involves only the active DOFs
    jacToJacReduced(jac,jac_reduced);
    if(use_nullspace)
    {
      for(int i=0; i < (int)chain.getNrOfJoints()-num_mimic_joints-num_redundant_joints; ++i)
      {
        q(i)=q_in(locked_joints_map_index[mimic_joints_[i].map_index]);
        q_comfortable_locked(i)=q_comfortable(locked_joints_map_index[mimic_joints_[i].map_index]);
      }
    }
    else
    {
      for(int i=0; i < (int)chain.getNrOfJoints()-num_mimic_joints-num_redundant_joints; ++i)
        q(i)=q_in(locked_joints_map_index[mimic_joints_[i].map_index]);
    }
  }
  else
  {
    jnt2jac.JntToJac(q_in,jac_reduced);
    if(use_nullspace)
    {
      for(int i=0; i < (int)chain.getNrOfJoints()-num_redundant_joints; ++i)
      {
        q(i)=q_in(locked_joints_map_index[mimic_joints_[i].map_index]);
        q_comfortable_locked(i)=q_comfortable(locked_joints_map_index[mimic_joints_[i].map_index]);
      }
    }
    else
    {
      for(int i=0; i < (int)chain.getNrOfJoints()-num_redundant_joints; ++i)
        q(i)=q_in(locked_joints_map_index[mimic_joints_[i].map_index]);
    }
  }

  //Now compute the jacobian with redundant joints locked
  jacToJacLocked(jac_reduced,jac_locked);

  Winv_locked.setZero();
  for(std::size_t i=0; i < chain.getNrOfJoints()-num_mimic_joints-num_redundant_joints; ++i)
    Winv_locked(i,i)=Winv(locked_joints_map_index[i],locked_joints_map_index[i]);

  switch(ik_type)
  {
    case kdl_kinematics_plugin::IKPositionOrientation6D:
      xd(0)=v_in.vel.x();
      xd(1)=v_in.vel.y();
      xd(2)=v_in.vel.z();
      xd(3)=v_in.rot.x();
      xd(4)=v_in.rot.y();
      xd(5)=v_in.rot.z();
      J=jac_locked.data;
      break;
    case kdl_kinematics_plugin::IKPosition3D:
      xd(0)=v_in.vel.x();
      xd(1)=v_in.vel.y();
      xd(2)=v_in.vel.z();
      J=jac_locked.data.topRows(3);
      break;
  }

  pinvJ = Winv_locked *J.transpose()*(C+J*Winv_locked*J.transpose()).inverse();

  if(use_nullspace)
  {
    qdot_out_reduced.data=(pinvJ*xd+(Eigen::MatrixXd::Identity(qdot_out_reduced.rows(),qdot_out_reduced.rows()) - pinvJ*J)*(q_comfortable_locked.data-q.data)*1e-3)*Kp;
  }
  else
  {
    qdot_out_reduced.data=pinvJ*xd*Kp;
  }


  ROS_DEBUG_STREAM_NAMED("kdl","Solution:");

  if(num_mimic_joints > 0)
  {
    for(int i=0; i < chain.getNrOfJoints(); ++i)
    {
      qdot_out(i) = qdot_out_reduced(locked_joints_map_index[mimic_joints_[i].map_index]) * mimic_joints_[i].multiplier;
    }
  }
  else
  {
    for(int i=0; i < chain.getNrOfJoints()-num_redundant_joints; ++i)
    {
      qdot_out(locked_joints_map_index[i]) = qdot_out_reduced(i);
    }
  }
  // Reset the flag
  // redundant_joints_locked = false;
  //return the return value of the svd decomposition
  return 0;
}

int ChainIkSolverVel_pinv_mimic_upgrade::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
  if(redundant_joints_locked)
    return CartToJntRedundant(q_in, v_in, qdot_out);

  Eigen::MatrixXd J;

  //Let the ChainJntToJacSolver calculate the jacobian "jac" for
  //the current joint positions "q_in". This will include the mimic joints

  q.resize(chain.getNrOfJoints()-num_mimic_joints);
  if(num_mimic_joints > 0)
  {
    //Now compute the actual jacobian that involves only the active DOFs
    jnt2jac.JntToJac(q_in,jac);
    jacToJacReduced(jac,jac_reduced);
    if(use_nullspace)
    {
      for(int i=0; i < (int)chain.getNrOfJoints()-num_mimic_joints; ++i)
      {
        q(i) = q_in(mimic_joints_[i].map_index);
        q_comfortable_reduced(i) = q_comfortable(mimic_joints_[i].map_index);
      }
    }
    else
    {
      for(int i=0; i < (int)chain.getNrOfJoints()-num_mimic_joints; ++i)
        q(i) = q_in(mimic_joints_[i].map_index);
    }
  }
  else
  {
    jnt2jac.JntToJac(q_in,jac_reduced);
    if(use_nullspace)
    {
      for(int i=0; i < chain.getNrOfJoints(); ++i)
      {
        q(i)=q_in(i);
        q_comfortable_reduced(i) = q_comfortable(i);
      }
    }
    else
    {
      for(int i=0; i < chain.getNrOfJoints(); ++i) q(i)=q_in(i);
    }
  }

  switch(ik_type)
  {
    case kdl_kinematics_plugin::IKPositionOrientation6D:
      xd(0)=v_in.vel.x();
      xd(1)=v_in.vel.y();
      xd(2)=v_in.vel.z();
      xd(3)=v_in.rot.x();
      xd(4)=v_in.rot.y();
      xd(5)=v_in.rot.z();
      J=jac_reduced.data;
      break;
    case kdl_kinematics_plugin::IKPosition3D:
      xd(0)=v_in.vel.x();
      xd(1)=v_in.vel.y();
      xd(2)=v_in.vel.z();
      J=jac_reduced.data.topRows(3);
      break;
  }

  pinvJ = Winv*J.transpose()*(C+J*Winv*J.transpose()).inverse();

  if(use_nullspace)
  {
    qdot_out_reduced.data=(pinvJ*xd+(Eigen::MatrixXd::Identity(qdot_out_reduced.rows(),qdot_out_reduced.rows()) - pinvJ*J)*(q_comfortable_reduced.data-q.data)*1e-3)*Kp;
  }
  else
  {
    qdot_out_reduced.data=pinvJ*xd*Kp;
  }

  ROS_DEBUG_STREAM_NAMED("kdl","Solution:");
  if(num_mimic_joints > 0)
  {
    for(int i=0; i < chain.getNrOfJoints(); ++i)
    {
      qdot_out(i) = qdot_out_reduced(mimic_joints_[i].map_index) * mimic_joints_[i].multiplier;
    }
  }
  else
    qdot_out = qdot_out_reduced;
  //return the return value of the svd decomposition

  return 0;
}
}
