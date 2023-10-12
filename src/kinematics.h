#pragma once

#include <BasicLinearAlgebra.h>
#include "utils.h"
#include <cmath>

struct KinematicsConfig
{
  float l1;
  float l2;
  float l3;
};

BLA::Matrix<4, 4> rotation_y(float theta)
{
  float rads = theta;
  BLA::Matrix<4, 4> rotation_mat = {cos(rads), 0, sin(rads), 0,
                      0, 1, 0, 0,
                      -sin(rads), 0, cos(rads), 0,
                      0, 0, 0, 1};
  return rotation_mat;
}

BLA::Matrix<4, 4> rotation_z(float theta)
{
  float rads = theta;
  BLA::Matrix<4, 4> rotation_mat = {cos(rads), -sin(rads), 0, 0,
                      sin(rads), cos(rads), 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1};
  
  return rotation_mat;
}

BLA::Matrix<4, 4> translation(float l1, float l2, float l3)
{
  BLA::Matrix<4, 4> translation_mat = {1, 0, 0, l1,
                      0, 1, 0, l2,
                      0, 0, 1, l3,
                      0, 0, 0, 1};
  return translation_mat;
}


// TODO: Step 12. Implement forward kinematics
BLA::Matrix<3> forward_kinematics(const BLA::Matrix<3> &joint_angles, const KinematicsConfig &config)
{
  /* Computes forward kinematics for the 3DOF robot arm/leg.
  
  Returns the cartesian coordinates of the end-effector corresponding to the given joint angles and leg configuration. 
  */

  /* Suggested Implementation
      Refer to Slide 38 in the FK Lecture Slides
      Parameters: Joint angles for each motor, config for the joint offsets
      Create helper functions to perform a rotation and translation together
        Parameters: theta, x, y, z
        Return: 4x4 Matrix for the corresponding translation (homogeneous coordinates)
      Call each transformation helper function together in this FK function, returning the cartesian coordinates in x, y, z
      Return: 3x1 Vector (BLA::Matrix<3,1>) for the x, y, z cartesian coordinates
  */ 
  BLA::Matrix<4, 4> identity_mat = {1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1};
  BLA::Matrix<4, 1> constant = {0, 0, config.l3, 1};
  BLA::Matrix<4, 1> res = identity_mat * rotation_z(joint_angles(0)) * translation(0, config.l1, 0) * rotation_y(joint_angles(1)) * 
  translation(0, 0, config.l2) * rotation_y(-joint_angles(2)) * constant;

  return BLA::Matrix<3>(res(0), res(1), res(2));
}

BLA::Matrix<3> inverse_kinematics(const BLA::Matrix<3> &target_location, const KinematicsConfig &config)
{
  return BLA::Matrix<3>(0, 0, 0);
}

enum class BodySide
{
  kLeft,
  kRight,
  kUnspecified
};

BLA::Matrix<3> correct_for_actuator_direction(const BLA::Matrix<3> &joint_vector, const BodySide &side)
{
  return {joint_vector(0), joint_vector(1), joint_vector(2)};
}
