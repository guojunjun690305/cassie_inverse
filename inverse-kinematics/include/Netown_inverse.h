#ifndef NETOWN_INVERSE_H_
#define NETOWN_INVERSE_H_
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "CRoboticConstraints.h"
#include "CCartesian.h"

using namespace std;
using namespace Eigen;

class Netown_inverse
{
    public:
        Netown_inverse();
        virtual ~Netown_inverse();
        void get_Angle(double* tChest, double* tAnkle, vector<double> &q, int whleg);

        
    private:
        vector<double> linkLength;
        double th_delt;
        CRoboticConstraints robot;
        CCartesian cu;
        CCartesian cart;

        MatrixXf e_pos;  // error between current pose and input pose
        MatrixXf pos_d; // Input cartesian x, y, z coordinates from the user
        MatrixXf e_pose;
        MatrixXf delt_R;
        MatrixXf ln_Rv;
        MatrixXf e;
        MatrixXf err;
        double theta_;

        MatrixXf i_1_theta;  // temporary joint angles variables in every iteration
        MatrixXf i_theta;    // final joint angles
        MatrixXf i_theta_Left;
        MatrixXf i_theta_Right;
        MatrixXf pose_d;

        MatrixXf X_roll;
        MatrixXf Y_pitch;
        MatrixXf Z_yaw;
        MatrixXf ZYX;

        MatrixXf pose_get;
        MatrixXf pose_get_transpose;
        MatrixXf pose;        // temporary pose variables in every iteration
        MatrixXf J;           // Jacobian

        double hiproll_offset;
        double hipyaw_offset;
        double hippitch_offset;//0.76794;
        double knee_offset;//1.3962634;
        double footpitch_offset;//0.62 ;
        double footroll_offset;//0.62 ;
        double anklepitch_offset;//1.2489576;

        

};

#endif
