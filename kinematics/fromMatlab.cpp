#include<iostream>
#include<Eigen/Dense>

struct dkin
{
    Eigen::Matrix3d rot;
    Eigen::Vector3d xyz;
};


void printMat(const Eigen::Matrix3d& m){
    std::cout << m << std::endl;
}

void printMat(const Eigen::Matrix4d& m){
    std::cout << m << std::endl;
}

/**
 * @brief Return the rotation matrix for the x axis 
 * 
 * @param alpha angle of the rotation
 * @return Eigen::Matrix3d 
 */
Eigen::Matrix3d xRot(float alpha){
    Eigen::Matrix3d xrot;
    xrot << 1, 0, 0,
            0, std::cos(alpha), -std::sin(alpha),
            0, std::sin(alpha), std::cos(alpha);
    return xrot;
}

/**
 * @brief Return the rotation matrix for the y axis
 * 
 * @param alpha angle of the rotation
 * @return Eigen::Matrix3d 
 */
Eigen::Matrix3d yRot(float alpha){
    Eigen::Matrix3d yrot;
    yrot << std::cos(alpha), 0, std::sin(alpha),
            0, 1, 0,
            -std::sin(alpha),0, std::cos(alpha);
    return yrot;
}

/**
 * @brief Return the rotation matrix for the z axis
 * 
 * @param alpha angle of the rotation
 * @return Eigen::Matrix3d 
 */
Eigen::Matrix3d zRot(float alpha){
    Eigen::Matrix3d zrot;
    zrot << cos(alpha), -sin(alpha), 0,
            sin(alpha), cos(alpha), 0,
            0,0,1;
    return zrot;
}

/**
 * @brief Return the euler rot matrix
 * 
 * @param phi angle of the z rotation
 * @param teta angle of the y rotation
 * @param psi angle of the z rotation
 * @return Eigen::Matrix3d 
 */
Eigen::Matrix3d eulerRot(float phi, float teta, float psi){
    Eigen::Matrix3d m;
    m = zRot(phi) * yRot(teta) * zRot(psi);
    return m;
}

/**
 * @brief Compute the Homogeneous Transform
 * 
 * @param phi 
 * @param teta 
 * @param psi 
 * @param ov 
 * @return Eigen::Matrix4d 
 */
Eigen::Matrix4d homogeneousTrans(float phi, float teta, float psi, const Eigen::Vector3d& ov){
    Eigen::Vector3d aux = ov;
    Eigen::Matrix3d m = eulerRot(phi, teta, psi);
    Eigen::Matrix4d t;
    t << m, ov,
         0, 0, 0, 1; 
    return t;
}

/**
 * @brief compute the direct kinematics
 * 
 * @param th joint angles
 * @return dkin : rot matrix and cartesian point (respect of the base frame i think)
 */
dkin directKin(float *th){
    dkin ret;
    //Vector of A distances (meters)
    float a[6] = {0, -4.25, -0.3922, 0, 0, 0};
    //Vector of D distances (meters)
    float d[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

    Eigen::Matrix4d t10;
    t10 << std::cos(th[0]), -std::sin(th[0]), 0, 0,
           std::sin(th[0]), std::cos(th[0]), 0, 0,
           0, 0, 1, d[0],
           0, 0, 0, 1;

    Eigen::Matrix4d t21;
    t21 << std::cos(th[1]), -std::sin(th[1]), 0, 0,
           0, 0, -1, 0,
           std::sin(th[1]), std::cos(th[1]), 0, 0,
           0, 0, 0, 1;

    Eigen::Matrix4d t32;
    t32 << std::cos(th[2]), -std::sin(th[2]), 0, a[1],
           std::sin(th[2]), std::cos(th[2]), 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;

    Eigen::Matrix4d t43;
    t43 << std::cos(th[3]), -std::sin(th[3]), 0, a[2],
           std::sin(th[3]), std::cos(th[3]), 0, 0,
           0, 0, 1, d[3],
           0, 0, 0, 1;

    Eigen::Matrix4d t54;
    t54 << std::cos(th[4]), -std::sin(th[4]), 0, 0,
           0, 0, -1, -d[4],
           std::sin(th[4]), std::cos(th[4]), 0, 0,
           0, 0, 0, 1;

    Eigen::Matrix4d t65;
    t65 << std::cos(th[5]), -std::sin(th[5]), 0, 0,
           0, 0, 1, d[5],
           -std::sin(th[5]), -std::cos(th[5]), 0, 0,
           0, 0, 0, 1;   

    Eigen::Matrix4d t60;

    t60 = t10*t21*t32*t43*t54*t65;

    //computation of ee position
    //cartesian position of the end effector (base frame POV)
    Eigen::Vector3d pos;
    pos(0) = t60(0, 3);
    pos(1) = t60(1, 3);
    pos(2) = t60(2, 3);
    //Rotation matrix of end effector
    Eigen::Matrix3d r;
    r(0,0) = t60(0,0);
    r(0,1) = t60(0,1);
    r(0,2) = t60(0,2);
    r(1,0) = t60(1,0);
    r(1,1) = t60(1,1);
    r(1,2) = t60(1,2);
    r(2,0) = t60(2,0);
    r(2,1) = t60(2,1);
    r(2,2) = t60(2,2);

    //return these 2 params
    ret.rot = r;
    ret.xyz = pos;

    return ret;
}

// Ancora da fare
/* void invKin(){
    //Vector of A distances (meters)
    float a[6] = {0, -4.25, -0.3922, 0, 0, 0};
    //Vector of D distances (meters)
    float d[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
} */

int main(){
    struct dkin p;
    float th[6] = {0, 0, 0, 0, 0, 0};
    p = directKin(th);
    std::cout << p.rot << std::endl << p.xyz << std::endl;
}