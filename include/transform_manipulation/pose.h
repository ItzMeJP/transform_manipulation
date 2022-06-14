/**\file
 * \brief
 * Pose class declaration.
 * @version 02.07.2021
 * @author João Pedro Carvalho de Souza
 */

#ifndef TRANSFORM_MANIPULATION_POSE_H
#define TRANSFORM_MANIPULATION_POSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h>

#define MSG_PREFIX "<TransformManipulation> "

#ifndef NDEBUG
#define DEBUG_MSG(str) do { std::cout << "\033[;33m" << MSG_PREFIX << str << "\033[0m"<< std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

class Pose{
public:

    /// <summary>
    /// Constructor
    /// </summary>
    Pose();

    /// <summary>
    /// Constructor with initialization.
    /// </summary>
    /// <param name="_name"> Frame name. </param>
    /// <param name="_parent_frame"> Parent frame name. </param>
    /// <param name="_position"> Position </param>
    /// <param name="_quaternion_orientation"> Orientation in quaternion </param>
    Pose(std::string _name,std::string _parent_frame, Eigen::Translation3d _position, Eigen::Quaterniond _quaternion_orientation);

    /// <summary>
    /// Constructor with initialization.
    /// </summary>
    /// <param name="_name"> Frame name. </param>
    /// <param name="_parent_frame"> Parent frame name. </param>
    /// <param name="_position"> Position </param>
    /// <param name="_quaternion_orientation"> Orientation in Radians in Tait-Bryan (or cumulative Euler) angle (ZYX sequence). </param>
    Pose(std::string _name,std::string _parent_frame,Eigen::Translation3d _position, Eigen::Vector3d _rpy_zyx_orientation);

    ~Pose();

    /// <summary>
    /// Get the frame stored quaternion angle (in radians) representation in Tait-Bryan (or cumulative Euler) angle (ZYX sequence).
    /// </summary>
    /// <returns> The frame orientation.</returns>
    Eigen::Vector3d getRPYOrientationZYXOrder();

    /// <summary>
    /// Get the frame stored orientation in quaternion format.
    /// </summary>
    /// <returns> The frame orientation in eigen quaternion format.</returns>
    Eigen::Quaterniond getQuaternionOrientation();

    /// <summary>
    /// Get the frame stored position
    /// </summary>
    /// <returns> The frame position.</returns>
    Eigen::Translation3d getPosition();

    /// <summary>
    /// Get the Eigen affine transformation object
    /// </summary>
    /// <returns> The affine object position.</returns>
    Eigen::Affine3d getAffine();

    /// <summary>
    /// Get the pose in format of transformation matrix [R(3x3) T(3,1); {0 0 0 1}]
    /// </summary>
    /// <returns> The transformation matrix [R(3x3) T(3,1); {0 0 0 1}].</returns>
    Eigen::Matrix4d getMatrix();

    /// <summary>
    /// Get the rotation matrix
    /// </summary>
    /// <returns> The 3x3 rotation matrix.</returns>
    Eigen::Matrix3d getRotationMatrix();

    /// <summary>
    /// Get the translation vector
    /// </summary>
    /// <returns> The 3x1 translation vector.</returns>
    Eigen::Vector3d getTranslationVector();

    /// <summary>
    /// Get the frame name
    /// </summary>
    /// <returns> The frame name.</returns>
    std::string getName();

    /// <summary>
    /// Get the frame parent name
    /// </summary>
    /// <returns> The frame parent name.</returns>
    std::string getParentName();

    /// <summary>
    /// Set frame orientation in roll(x-axis), pitch(y-axis) and yaw(z-axis) angles. Tait-Bryan (or cumulative Euler) angle in ZYX sequence.
    /// </summary>
    /// <param name="_in"> Eigen vector array object in radians- </param>
    void setRPYOrientationZYXOrder(Eigen::Vector3d _in);

    /// <summary>
    /// Set frame quaternion orientation
    /// </summary>
    /// <param name="_in"> Eigen quaternion object.</param>
    void setQuaternionOrientation(Eigen::Quaterniond _in);

    /// <summary>
    /// Set frame position
    /// </summary>
    /// <param name="_in"> Eigen translation 3D object.</param>
    void setPosition(Eigen::Translation3d _in);

    /// <summary>
    /// Set frame name (child name)
    /// </summary>
    /// <param name="_in"> Frame name.</param>
    void setName(std::string _in);

    /// <summary>
    /// Set frame parent name
    /// </summary>
    /// <param name="_in"> Frame parent name.</param>
    void setParentName(std::string _in);


    // ********** Static methods definitions ********** //


    /// <summary>
    /// Transform a Euler angle representation (cumulative or not) to quaternion angle representation. The construction sequence is ZYX
    /// </summary>
    /// <param name="_v"> the Vector3 with Roll(x), Pitch(y) and Yaw(z) angles.</param>
    /// <returns> The quaternion representation angles in eigen vector 3d format.</returns>
    static Eigen::Quaterniond getQuaternionFromZYXEuler(Eigen::Vector3d _v);

    /// <summary>
    /// Transform a quaternion angle representation to Tait-Bryan (or cumulative Euler) angle. The construction sequence is ZYX
    /// </summary>
    /// <param name="_q"> Quaternion.</param>
    /// <returns> The Vector3 with Roll(x), Pitch(y) and Yaw(z) angles.</returns>
    static Eigen::Vector3d getTaitBryanZYXFromQuaternion(Eigen::Quaterniond _q);

    /// <summary>
    /// Transform a quaternion angle representation to Tait-Bryan (or cumulative Euler) angle. The construction sequence is ZYX
    /// </summary>
    /// <param name="_q"> Quaternion.</param>
    /// <returns> The Vector3 with Roll(x), Pitch(y) and Yaw(z) angles in degrees.</returns>
    static Eigen::Vector3d getTaitBryanZYXFromQuaternionInDegrees(Eigen::Quaterniond _q);

    /// <summary>
    /// Transform a polar rotation (Azimuth and Polar angles) into a Rotation Matrix
    /// </summary>
    /// <param name="_azimuth_angle"> Azimuth angle in radians (equatorial plane).</param>
    /// <param name="_polar_angle"> Polar angle in radians (meridian plane).</param>
    /// <returns> The rotation 3x3 matrix</returns>
    static Eigen::Matrix3d getRotationMatrixFromPolarCoordinate(double _azimuth_angle, double _polar_angle);

    /// <summary>
    /// Transform a rotation matrix into quaternion.
    /// </summary>
    /// <param name="_m"> Rotation matrix.</param>
    /// <returns> The Quaternion type with w,x,y,z coefs.</returns>
    static Eigen::Quaterniond getQuaternionFromRotMatrix(Eigen::Matrix3d _m);

    /// <summary>
    /// Signal function.
    /// </summary>
    /// <param name="_in"> Input value.</param>
    /// <returns> -1 if _in < 0 , +1 if _in > 0 and 0 if _in = 0</returns>
    static double sgnd(double _in);

    /// <summary>
    /// Convert radians to degree.
    /// </summary>
    /// <param name="_radians"> Angle in radians</param>
    /// <returns> the angles in degree </returns>
    static double convertRadToDeg(double _radians);

protected:
    std::string name_, parent_frame_;
    Eigen::Translation3d position_;
    Eigen::Vector3d rpy_zyx_orientation_;
    Eigen::Quaterniond quaternion_orientation_;
    Eigen::Affine3d affine_;
    Eigen::Matrix4d transform_matrix_;
    Eigen::Matrix3d rotation_matrix_;
    Eigen::Vector3d translation_vector_;

    /// <summary>
    /// Generate the transformation matrix (4x4) based on the pose/frame.
    /// </summary>
    void buildMatrix();


};

#endif //TRANSFORM_MANIPULATION_POSE_H
