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
