/**\file
 * \brief
 * Pose class declaration.
 * @version 03.09.2021
 * @author João Pedro Carvalho de Souza
 */

#ifndef TRANSFORM_MANIPULATION_TRANSFORM_H
#define TRANSFORM_MANIPULATION_TRANSFORM_H

#include "pose.h"

class Transform : public Pose {

public:
    Transform(std::string _name, std::string _parent_frame, Eigen::Translation3d _position, Eigen::Quaterniond _quaternion_orientation);
    Transform(std::string _name, std::string _parent_frame, Eigen::Translation3d _position, Eigen::Vector3d _rpy_zyx_orientation);
    Transform(Pose _p);
    Transform(std::string _name, std::string _parent_frame, Eigen::Matrix4d _m);

    ~Transform();

    bool apply(Pose _target, Pose &_output);
    bool apply_inverse(Pose _target, Pose &_output);




};

#endif //TRANSFORM_MANIPULATION_TRANSFORM_H
