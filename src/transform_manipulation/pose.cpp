/**\file
 * \brief
 * Pose class definition.
 * @version 31.05.2022
 * @author João Pedro Carvalho de Souza
 */

#include "pose.h"

Pose::Pose(){
    name_ = "NULL";
    parent_frame_ = "NULL";
    position_ = Eigen::Translation3d(0,0,0);
    rpy_zyx_orientation_ = Eigen::Vector3d(0,0,0);
    Eigen::Quaterniond quaternion_orientation_= Eigen::Quaterniond (1,0,0,0);

    affine_ = position_*quaternion_orientation_;
    buildMatrix();


};

Pose::Pose(std::string _name, std::string _parent_frame, Eigen::Translation3d _position,
           Eigen::Quaterniond _quaternion_orientation) {
    name_ = _name;
    parent_frame_ = _parent_frame;
    position_ = _position;
    quaternion_orientation_ = _quaternion_orientation;
    //rpy_zyx_orientation_ = quaternion_orientation_.matrix().eulerAngles(2,0,2); //ZYZ
    rpy_zyx_orientation_ = getTaitBryanZYXFromQuaternion(quaternion_orientation_);

    affine_ = position_*quaternion_orientation_;

    buildMatrix();

}

Pose::Pose(std::string _name, std::string _parent_frame, Eigen::Translation3d _position,
           Eigen::Vector3d _rpy_zyx_orientation) {
    name_ = _name;
    parent_frame_ = _parent_frame;
    position_ = _position;
    rpy_zyx_orientation_ = _rpy_zyx_orientation;
    quaternion_orientation_ = getQuaternionFromZYXEuler(rpy_zyx_orientation_);

    affine_ = position_*quaternion_orientation_;

    buildMatrix();


}

Pose::~Pose() {};

Eigen::Vector3d Pose::getRPYOrientationZYXOrder() {
    return rpy_zyx_orientation_;
}

Eigen::Quaterniond Pose::getQuaternionOrientation() {
    return quaternion_orientation_;
}

Eigen::Translation3d Pose::getPosition(){
    return position_;
}

Eigen::Affine3d Pose::getAffine(){
    return affine_;
}


std::string Pose::getName(){
    return name_;
}

std::string Pose::getParentName(){
    return parent_frame_;
}

void Pose::setRPYOrientationZYXOrder(Eigen::Vector3d _in){
    rpy_zyx_orientation_ = _in;
    quaternion_orientation_ = getQuaternionFromZYXEuler(rpy_zyx_orientation_);
    affine_ = position_*quaternion_orientation_;
    buildMatrix();
}
void Pose::setQuaternionOrientation(Eigen::Quaterniond _in){
    quaternion_orientation_ = _in;
    rpy_zyx_orientation_ = getTaitBryanZYXFromQuaternion(quaternion_orientation_);
    affine_ = position_*quaternion_orientation_;
    buildMatrix();
}
void Pose::setPosition(Eigen::Translation3d _in){
    position_ = _in;
    affine_ = position_*quaternion_orientation_;
    buildMatrix();
}
void Pose::setName(std::string _in){
    name_ = _in;

}
void Pose::setParentName(std::string _in){
    parent_frame_ = _in;
}

Eigen::Matrix4d Pose::getMatrix(){
    return transform_matrix_;
}

/// <summary>
/// Get the translation vector
/// </summary>
/// <returns> The 3x1 translation vector.</returns>
Eigen::Matrix3d Pose::getRotationMatrix(){
    return rotation_matrix_;
}

/// <summary>
/// Get the translation vector
/// </summary>
/// <returns> The 3x1 translation vector.</returns>
Eigen::Vector3d Pose::getTranslationVector(){
    return translation_vector_;
}


void Pose::buildMatrix() {
    rotation_matrix_= affine_.rotation();
    translation_vector_ = affine_.translation();
    transform_matrix_.setIdentity();
    transform_matrix_.block<3,3>(0,0) = rotation_matrix_;
    transform_matrix_.block<3,1>(0,3) = translation_vector_;

}

/// <summary>
/// Transform a Euler angle representation (cumulative or not) to quaternion angle representation. The construction sequence is ZYX
/// </summary>
/// <param name="_v"> the Vector3 with Roll(x), Pitch(y) and Yaw(z) angles.</param>
/// <returns> The quaternion representation angles.</returns>
Eigen::Quaterniond Pose::getQuaternionFromZYXEuler(Eigen::Vector3d _v){
    Eigen::Quaterniond output;
    output = Eigen::AngleAxisd(_v.z(), Eigen::Vector3d::UnitZ())
                          * Eigen::AngleAxisd(_v.y(), Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(_v.x(), Eigen::Vector3d::UnitX());
    return output;
}

/// <summary>
/// Transform a rotation matrix into quaternion.
/// </summary>
/// <param name="_m"> Rotation matrix.</param>
/// <returns> The Quaternion type with w,x,y,z coefs.</returns>
Eigen::Quaterniond Pose::getQuaternionFromRotMatrix(Eigen::Matrix3d _m){
    Eigen::Quaterniond q(_m);
    return q;
}

/// <summary>
/// Transform a polar rotation (Azimuth and Polar angles) into a Rotation Matrix
/// </summary>
/// <param name="_azimuth_angle"> Azimuth angle in radians (equatorial plane).</param>
/// <param name="_polar_angle"> Polar angle in radians (meridian plane).</param>
/// <returns> The rotation 3x3 matrix</returns>
Eigen::Matrix3d Pose::getRotationMatrixFromPolarCoordinate(double _azimuth_angle, double _polar_angle){

    Eigen::Matrix3d r;
    r << cos(_azimuth_angle),      sin(_azimuth_angle)*cos(_polar_angle),      sin(_azimuth_angle)* sin(_polar_angle),
        -sin(_azimuth_angle),      cos(_azimuth_angle)* cos(_polar_angle),     cos(_azimuth_angle)* sin(_polar_angle),
                 0,                  -sin(_polar_angle),                             cos(_polar_angle);
    return r;
}


/// <summary>
/// Transform a quaternion angle representation to Tait-Bryan (or cumulative Euler) angle. The construction sequence is ZYX
/// </summary>
/// <param name="_q"> Quaternion.</param>
/// <returns> The Vector3 with Roll(x), Pitch(y) and Yaw(z) angles in radians.</returns>
Eigen::Vector3d Pose::getTaitBryanZYXFromQuaternion(Eigen::Quaterniond _q) {
    //https://marc-b-reynolds.github.io/math/2017/04/18/TaitEuler.html

    double x = _q.x(),
            y = _q.y(),
            z = _q.z(),
            w = _q.w();

    Eigen::Vector3d output;

    double t0 = x * x - z * z,
           t1 = w * w - y * y,
           xx = 0.5 * (t0 + t1),            // 1/2 x of x'
           xy = x * y + w * z,              // 1/2 y of x'
           xz = w * y - x * z,              // 1/2 z of x'
           t = xx * xx + xy * xy,           // cos(theta)^2
           yz = 2.0 * (y * z + w * x);      // z of y'

    output.z() = (float) atan2(xy, xx);         // yaw   (psi)
    output.y() = (float) atan(xz / sqrt(t)); // pitch (theta)

    if (t != 0)
        output.x() = (float) atan2(yz, t1 - t0);
    else
        output.x() = (float) (2.0 * atan2(x, w) - sgnd(xz) * output.z());

    return output;
}

/// <summary>
/// Transform a quaternion angle representation to Tait-Bryan (or cumulative Euler) angle. The construction sequence is ZYX
/// </summary>
/// <param name="_q"> Quaternion.</param>
/// <returns> The Vector3 with Roll(x), Pitch(y) and Yaw(z) angles in degrees.</returns>
Eigen::Vector3d Pose::getTaitBryanZYXFromQuaternionInDegrees(Eigen::Quaterniond _q){

    Eigen::Vector3d  radians_arr, deg_arr;
    radians_arr = getTaitBryanZYXFromQuaternion(_q);
    deg_arr <<  Pose::convertRadToDeg(radians_arr[0]) , Pose::convertRadToDeg(radians_arr[1]), Pose::convertRadToDeg(radians_arr[2]);
    return deg_arr;

}

double Pose::sgnd(double _in) {
    return (_in > 0.0) ? 1.0 : ((_in < 0.0) ? -1.0 : 0.0);
}

double Pose::convertRadToDeg(double _radians)
{
    return _radians * (180.0 / M_PI);
}




