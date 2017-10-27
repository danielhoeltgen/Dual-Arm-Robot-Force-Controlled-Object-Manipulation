//
// Created by Daniel Höltgen on 17.10.16.
//

#include "dual_arm_toolbox/Transform.h"

using namespace dual_arm_toolbox;

void Transform::transformPoseToKDL(geometry_msgs::Pose pose, KDL::Frame& kdl_frame){
    kdl_frame.M = kdl_frame.M.Quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
    );
    kdl_frame.p.x(pose.position.x);
    kdl_frame.p.y(pose.position.y);
    kdl_frame.p.z(pose.position.z);
}

void Transform::transformKDLtoPose(KDL::Frame kdl_frame, geometry_msgs::Pose& pose){
    kdl_frame.M.GetQuaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
    );
    pose.position.x = kdl_frame.p.x();
    pose.position.y = kdl_frame.p.y();
    pose.position.z = kdl_frame.p.z();
}