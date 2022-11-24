#include <iostream>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;

typedef Matrix <float, 1, 6> Array6d;

void ur5direct(Array6d Th, Matrix3d & rot, Vector3d & xyz){

    Array6d var;

    float A[6]= {0, -0.425, -0.3922, 0, 0, 0};
    float D[6]= {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

    //operational matrices
    Matrix4d T10, T21, T32, T43, T54, T65, T60;

    T10 << 
	 cos(Th(0)), -sin(Th(0)), 0, 0,
	 sin(Th(0)), cos(Th(0)), 0, 0,
	 0, 0, 1, D[0],
	 0, 0, 0, 1;
    T21  <<
	 cos(Th(1)), -sin(Th(1)), 0, 0,
	 0, 0, -1, 0,
	 sin(Th(1)), cos(Th(1)), 0, 0,
	 0, 0, 0, 1;
    T32 <<
	 cos(Th(2)), -sin(Th(2)), 0, A [1],
	 sin(Th(2)), cos(Th(2)), 0, 0,
	 0, 0, 1, 0,
	 0, 0, 0, 1;
    T43 <<
	 cos(Th(3)), -sin(Th(3)), 0, A[2],
	 sin(Th(3)), cos(Th(3)), 0, 0,
	 0, 0, 1, D[3],
	 0, 0, 0, 1;
    T54 <<
	 cos(Th(4)), -sin(Th(4)), 0, 0,
	 0, 0, -1, -D[4],
	 sin(Th(4)), cos(Th(4)), 0, 0,
	 0, 0, 0, 1;
    T65 <<
	 cos(Th(5)), -sin(Th(5)), 0, 0,
	 0, 0, 1, D[5],
	 -sin(Th(5)), -cos(Th(5)), 0, 0,
	 0, 0, 0, 1;

    T60 = T10*T21*T32*T43*T54*T65;
	cout << T60 << endl;

    //end effector position
	for (int i = 0; i< 3; i++){
		xyz(i)=T60(i,3);	
	}
	//rot matrix
    for (int i = 0; i<3; i++){
		for (int j= 0; j<3; j++){
			rot (i,j)= T60(i,j);
		}
	}
	
	


}
/*
void ur5inverse(Array6d Th, Matrix3d & rot, Vector3d xyz){

    Array6d var;

    float A[6]= {0, -0.425, -0.3922, 0, 0, 0};
    float D[6]= {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

    //operational matrices
    Matrix4d T10, T21, T32, T43, T54, T65, T60;

    T10 << 
	 cos(Th(0)), -sin(Th(0)), 0, 0,
	 sin(Th(0)), cos(Th(0)), 0, 0,
	 0, 0, 1, D[0],
	 0, 0, 0, 1;
    T21  <<
	 cos(Th(1)), -sin(Th(1)), 0, 0,
	 0, 0, -1, 0,
	 sin(Th(1)), cos(Th(1)), 0, 0,
	 0, 0, 0, 1;
    T32 <<
	 cos(Th(2)), -sin(Th(2)), 0, A [1],
	 sin(Th(2)), cos(Th(2)), 0, 0,
	 0, 0, 1, 0,
	 0, 0, 0, 1;
    T43 <<
	 cos(Th(3)), -sin(Th(3)), 0, A[2],
	 sin(Th(3)), cos(Th(3)), 0, 0,
	 0, 0, 1, D[3],
	 0, 0, 0, 1;
    T54 <<
	 cos(Th(4)), -sin(Th(4)), 0, 0,
	 0, 0, -1, -D[4],
	 sin(Th(4)), cos(Th(4)), 0, 0,
	 0, 0, 0, 1;
    T65 <<
	 cos(Th(5)), -sin(Th(5)), 0, 0,
	 0, 0, 1, D[5],
	 -sin(Th(5)), -cos(Th(5)), 0, 0,
	 0, 0, 0, 1;

}


*/

int main (){

    Array6d th;
    th << 0, 0, 0, 0, 0, 0;

	Matrix3d rot;
	Vector3d xyz;

    ur5direct (th, rot, xyz);
	cout << xyz << endl;
	cout << rot << endl;
    cout << "fine" << endl;

    return 0;
}