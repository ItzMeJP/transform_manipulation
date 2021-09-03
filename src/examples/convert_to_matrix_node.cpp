//
// Created by joaopedro on 03/09/21.
//

//
// Created by joaopedro on 03/09/21.
//

#include "transform.h"

int main(){

    Eigen::Translation3d t_aux(1,-2,4);
    Eigen::Vector3d v_aux(1.0,-0.1,-.12);
    Pose p("generic_pose","original",t_aux,v_aux);

    std::cout << "\nQuaternion format [x,y,z,w]:\n" << std::endl;
    std::cout << p.getQuaternionOrientation().x() << " | " << p.getQuaternionOrientation().y() << " | " << p.getQuaternionOrientation().z() << " | " << p.getQuaternionOrientation().w() << std::endl;
    std::cout << "\nTait-Bryan (RPY - ZYX) format [roll,pitch,yaw]:\n" << std::endl;
    std::cout << p.getRPYOrientationZYXOrder() << std::endl;
    std::cout << "\nMatrix format:\n" << std::endl;
    std::cout << p.getMatrix() << std::endl;


    return 0;

}
