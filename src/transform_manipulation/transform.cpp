/**\file
 * \brief
 * Pose class definition.
 * @version 03.09.2021
 * @author João Pedro Carvalho de Souza
 */

#include "transform.h"


Transform::Transform(std::string _name, std::string _parent_frame, Eigen::Matrix4d _m){

    affine_ = _m;

    Eigen::Matrix3d m = affine_.rotation();
    Eigen::Vector3d v = affine_.translation();

    Eigen::Translation3d translation(v);
    Eigen::Quaterniond quaternion_orientation(m);

    name_ = _name;
    parent_frame_ = _parent_frame;
    position_ = translation;
    quaternion_orientation_ = quaternion_orientation;
    rpy_zyx_orientation_ = getTaitBryanZYXFromQuaternion(quaternion_orientation_);

    buildMatrix();
}

Transform::Transform(std::string _name, std::string _parent_frame, Eigen::Translation3d _position, Eigen::Quaterniond _quaternion_orientation){
    name_ = _name;
    parent_frame_ = _parent_frame;
    position_ = _position;
    quaternion_orientation_ = _quaternion_orientation;
    //rpy_zyx_orientation_ = quaternion_orientation_.matrix().eulerAngles(2,0,2); //ZYZ
    rpy_zyx_orientation_ = getTaitBryanZYXFromQuaternion(quaternion_orientation_);

    affine_ = position_*quaternion_orientation_;
    buildMatrix();
}

Transform::Transform(std::string _name, std::string _parent_frame, Eigen::Translation3d _position, Eigen::Vector3d _rpy_zyx_orientation){
    name_ = _name;
    parent_frame_ = _parent_frame;
    position_ = _position;
    rpy_zyx_orientation_ = _rpy_zyx_orientation;
    quaternion_orientation_ = getQuaternionFromZYXEuler(rpy_zyx_orientation_);

    affine_ = position_*quaternion_orientation_;
    buildMatrix();
}

Transform::Transform(Pose _p){
    name_ = _p.getName();
    parent_frame_ = _p.getParentName();
    position_ = _p.getPosition();
    rpy_zyx_orientation_ = _p.getRPYOrientationZYXOrder();
    quaternion_orientation_ = _p.getQuaternionOrientation();

    affine_ = position_*quaternion_orientation_;
    buildMatrix();
}

Transform::~Transform() {

}

bool Transform::apply(Pose _target, Pose &_output){

    if(name_.compare(_target.getParentName()) != 0 ){
        DEBUG_MSG("Transformation from \"" << name_ << "\" to \""<< parent_frame_ << "\" does not match the parent target \"" << _target.getParentName() << "\".");
        return false;
    }

    Eigen::Affine3d result_affine = affine_*_target.getAffine();

    Eigen::Matrix3d m = result_affine.rotation();
    Eigen::Vector3d v = result_affine.translation();

    Eigen::Translation3d translation(v);
    Eigen::Quaterniond quaternion_orientation(m);

    _output.setName(_target.getName());
    _output.setParentName(parent_frame_);
    _output.setPosition(translation);
    _output.setQuaternionOrientation(quaternion_orientation);

     return true;
}

bool Transform::apply_inverse(Pose _target, Pose &_output){

    if(parent_frame_.compare(_target.getParentName()) != 0 ){
        DEBUG_MSG("Transformation from \"" << parent_frame_ << "\" to \""<< name_ << "\" does not match the parent target \"" << _target.getParentName() << "\".");
        return false;
    }

    Eigen::Affine3d result_affine = affine_.inverse()*_target.getAffine();
    Eigen::Matrix3d m = result_affine.rotation();
    Eigen::Vector3d v = result_affine.translation();

    Eigen::Translation3d translation(v);
    Eigen::Quaterniond quaternion_orientation(m);

    _output.setName(_target.getName());
    _output.setParentName(name_);
    _output.setPosition(translation);
    _output.setQuaternionOrientation(quaternion_orientation);

    return true;
}

