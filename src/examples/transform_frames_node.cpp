//
// Created by joaopedro on 03/09/21.
//

#include "transform.h"

int main(){

    Eigen::Matrix4d m,id, rotz45, rotx77;

    id
      << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    rotz45
      << 0.7071, -0.7071, 0, 0,
         0.7071,  0.7071, 0, 0,
         0     ,       0, 1, 0,
         0     ,       0, 0, 1;

    rotx77
      << 1, 0,0,0,
      0,0.225,-0.9744,0,
      0,0.9744,0.2250,0,
      0,0,0,1;

    m = rotx77;

    Transform t("original","new_base",m);

    std::cout << "\nTransformation matrix of \""<< t.getName() << "\" referred to \""<< t.getParentName() << "\""<< std::endl;
    std::cout << t.getMatrix() << std::endl;

    Eigen::Translation3d t_aux(1,-2,4);
    Eigen::Vector3d v_aux(0,0,1.15);

    Pose p("generic_pose","original",t_aux,v_aux),
          p_transformed;

    if(!t.apply(p,p_transformed))
        return 1;

    std::cout << "\nInput frame: " << std::endl;
    std::cout << "Frame name: " << p.getName() << std::endl;
    std::cout << "Parent Frame: " << p.getParentName() << std::endl;
    std::cout << "Position [x,y,z]:\n" << p.getPosition().x() << "|"
              << p.getPosition().y() << "|"
              << p.getPosition().z() << std::endl;
    std::cout << "Orientation [x,y,z]:\n" << p.getRPYOrientationZYXOrder().x() << "|"
              << p.getRPYOrientationZYXOrder().y() << "|"
              << p.getRPYOrientationZYXOrder().z() << std::endl;
    std::cout << "Matrix format:\n" << std::endl;
    std::cout << p.getMatrix() << std::endl;

    std::cout << "\nTransformed frame: " << std::endl;
    std::cout << "Frame name: " << p_transformed.getName() << std::endl;
    std::cout << "Parent Frame: " << p_transformed.getParentName() << std::endl;
    std::cout << "Position [x,y,z]:\n" << p_transformed.getPosition().x()<< "|"
            << p_transformed.getPosition().y()<< "|"
            << p_transformed.getPosition().z() << std::endl;
    std::cout << "Orientation [x,y,z]:\n" << p_transformed.getRPYOrientationZYXOrder().x()<< "|"
            << p_transformed.getRPYOrientationZYXOrder().y()<< "|"
            << p_transformed.getRPYOrientationZYXOrder().z() << std::endl;
    std::cout << "Matrix format:\n" << std::endl;
    std::cout << p_transformed.getMatrix() << std::endl;

    return 0;

}
