#pragma once
#ifndef UR_KIN_H
#define UR_KIN_H
#define M_PI 3.14159265358979323846 
// These kinematics find the tranfrom from the base link to the end effector.
// Though the raw D-H parameters specify a transform from the 0th link to the 6th link,
// offset transforms are specified in this formulation.
// To work with the raw D-H kinematics, use the inverses of the transforms below.

// Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1
using namespace std;

namespace ur_kinematics {
	// @param q       The 6 joint values 
	// @param T       The 4x4 end effector pose in row-major ordering
	void forward(const double* q, double* T);

	// @param q       The 6 joint values 
	// @param Ti      The 4x4 link i pose in row-major ordering. If NULL, nothing is stored.
	void forward_all(const double* q, double* T1, double* T2, double* T3,
		double* T4, double* T5, double* T6);

	// @param T       The 4x4 end effector pose in row-major ordering
	// @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
	// @param q6_des  An optional parameter which designates what the q6 value should take
	//                in case of an infinite solution on that joint.
	// @return        Number of solutions found (maximum of 8)
	int inverse(const double* T, double* q_sols, double q6_des = 0.0);

    void qSelect(double* q_ori, double* q_cri,double* q_res);//6*6  1*6  1*6

	double distance_(double* q1, double* q2);//1*6  1*6

	void qNorm(double* q);//i*6
}

#endif //UR_KIN_H


//#ifndef IKFAST_NO_MAIN

//using namespace std;
//using namespace ur_kinematics;

//int main(int argc, char* argv[])
//{
//	//double q[6] = { 0.0, 0.0, 1.0, 0.0, 1.0, 0.0 };
//	double q[6] = { 2.0, 3.0, 4.0, 1.0, 1.5, 5.0 };
//	double* T = new double[16];
//	forward(q, T);
//	for (int i = 0; i < 4; i++) {
//		for (int j = i * 4; j < (i + 1) * 4; j++)
//			printf("%1.3f ", T[j]);
//		printf("\n");
//	}
//	double q_sols[8 * 6];
//	int num_sols;
//	num_sols = inverse(T, q_sols);
//	for (int i = 0; i < num_sols; i++)
//		printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n",
//			q_sols[i * 6 + 0], q_sols[i * 6 + 1], q_sols[i * 6 + 2], q_sols[i * 6 + 3], q_sols[i * 6 + 4], q_sols[i * 6 + 5]);

//	double q_res[6];
//	double q_cri[6] = { 2.0, 3.0, 4.0, -3.0, 4.0, 1.0 };
//	qNorm(q_cri);
//	printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n",
//		q_cri[0], q_cri[1], q_cri[2], q_cri[3], q_cri[4], q_cri[5]);
//	qSelect(q_sols, q_cri, q_res);//8*6  1*6  1*6
//	printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n",
//		q_res[0], q_res[1], q_res[2], q_res[3], q_res[4], q_res[5]);
//	//for (int i = 0; i <= 4; i++)
//	//	printf("%f ", PI / 2.0*i);
//	//printf("\n");
//	return 0;
//}
//#endif
