//
// Created by joaopedro on 08/06/22.
//

#include "transform.h"
#include <fstream>


using std::ifstream;

bool processFromMatrix(std::string _path) {

    const int m = 3, n = 3;
    double  matrix[m][n];
    Eigen::Matrix3d matrix_e;
    //std::stringstream ss;

    ifstream indata(_path);

    if (!indata.is_open()) {
        std::cerr << "Error: file could not be opened" << std::endl;
        return false;
    }

    for (int i = 0; i < m; i++)
        for (int j = 0; j < m; j++) {
            indata >> matrix[i][j];
            //ss << matrix[i][j] << " ";
        }

    matrix_e << matrix[0][0], matrix[0][1],matrix[0][2],
                matrix[1][0],matrix[1][1],matrix[1][2],
                matrix[2][0],matrix[2][1],matrix[2][2];

    std::cout << " ------------------------------------- Direct Matrix Input -----------------------------------------------------" << std::endl;

    std::cout << "Input Matrix:" << std::endl;
    std::cout << matrix_e << "\n" << std::endl;
    //std::cout << "Input Matrix values:\n[" << ss.str() << "]\nEnd-of-file reached..\n" << std::endl;

    indata.close();

    Eigen::Quaterniond q = Pose::getQuaternionFromRotMatrix(matrix_e) ;
    Eigen::Vector3d angles_rad =  Pose::getTaitBryanZYXFromQuaternion(q);
    Eigen::Vector3d angles_deg =  Pose::getTaitBryanZYXFromQuaternionInDegrees(q);

    std::cout << "RPY [Roll, Pitch, Yaw][rad]:    "<< angles_rad.x() << ", " << angles_rad.y() << ", " << angles_rad.z() << std::endl;
    std::cout << "RPY [Roll, Pitch, Yaw][deg]:    "<< angles_deg.x() << ", " << angles_deg.y() << ", " << angles_deg.z()  << std::endl;
    std::cout << "Quaternion [x, y, z, w]:        "<< q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()  << "\n" <<std::endl;
    std::cout << " --------------------------------------------------------------------------------------------------" << std::endl;

    std::cout << " ------------------------------------- Inverse Matrix Input -----------------------------------------------------" << std::endl;
    std::cout << "Input Matrix:" << std::endl;
    std::cout << matrix_e.inverse() << "\n" << std::endl;

    q = Pose::getQuaternionFromRotMatrix(matrix_e.inverse()) ;
    angles_rad =  Pose::getTaitBryanZYXFromQuaternion(q);
    angles_deg =  Pose::getTaitBryanZYXFromQuaternionInDegrees(q);

    std::cout << "RPY [Roll, Pitch, Yaw][rad]:    "<< angles_rad.x() << ", " << angles_rad.y() << ", " << angles_rad.z() << std::endl;
    std::cout << "RPY [Roll, Pitch, Yaw][deg]:    "<< angles_deg.x() << ", " << angles_deg.y() << ", " << angles_deg.z()  << std::endl;
    std::cout << "Quaternion [x, y, z, w]:        "<< q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()  << "\n" <<std::endl;
    std::cout << " --------------------------------------------------------------------------------------------------" << std::endl;
    return true;
}

bool processFromQuat(std::string _path) {

    const int m = 4;
    double  matrix[m];
    Eigen::Quaterniond q;

    ifstream indata(_path);

    if (!indata.is_open()) {
        std::cerr << "Error: file could not be opened" << std::endl;
        return false;
    }

    for (int i = 0; i < m; i++)
        indata >> matrix[i];


    q.x() = matrix[0];
    q.y() = matrix[1];
    q.z() = matrix[2];
    q.w() = matrix[3];

    std::cout << "Input quaternion [x,y,z,w]:" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()  << "\n" <<std::endl;

    indata.close();

    Eigen::Vector3d angles_rad =  Pose::getTaitBryanZYXFromQuaternion(q);
    Eigen::Vector3d angles_deg =  Pose::getTaitBryanZYXFromQuaternionInDegrees(q);

    Pose p;
    p.setQuaternionOrientation(q);
    Eigen::Matrix3d matrix_e = p.getRotationMatrix();


    std::cout << "RPY [Roll, Pitch, Yaw][rad]:    "<< angles_rad.x() << ", " << angles_rad.y() << ", " << angles_rad.z() << std::endl;
    std::cout << "RPY [Roll, Pitch, Yaw][deg]:    "<< angles_deg.x() << ", " << angles_deg.y() << ", " << angles_deg.z()  << std::endl;
    std::cout << "Rotation Matrix:" << std::endl;
    std::cout << matrix_e << "\n" << std::endl;
    return true;
}

int main(int argc, char **argv){

    if(argc<=1) {
        std::cerr << "None input args detected. Type -h for help menu." << std::endl;
        return false;
    }
    else{
        //std::cout << argv[1] << std::endl;

        if(strcmp(argv[1], "-h") == 0){
            std::cout << "Arguments: \n" << std::endl;
            std::cout << "   -m: path to rotation matrix file; \n" << std::endl;
            std::cout << "   -q: path to rotation quaternion file [x,y,z,w]; \n" << std::endl;

            return true;
        }

        else if(strcmp(argv[1], "-m") == 0){
            if(argc<=2){
                std::cerr << "Missing input file. " << std::endl;
                return false;
            }
            std::cout << "Converting rotation matrix from input file: " << argv[2] <<" \n" << std::endl;
            if(!processFromMatrix(argv[2])){
                std::cerr<<"Error in converting from matrix." << std::endl;
                return false;
            }
        }

        else if(strcmp(argv[1], "-q") == 0){
            if(argc<=2){
                std::cerr << "Missing input file. " << std::endl;
                return false;
            }
            std::cout << "Converting quaternion form input file: " << argv[2] <<" \n" << std::endl;
            if(!processFromQuat(argv[2])){
                std::cerr<<"Error in converting from quaternion." << std::endl;
                return false;
            }
        }
    }








    return 0;

}
