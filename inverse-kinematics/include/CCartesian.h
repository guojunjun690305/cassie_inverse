#ifndef CCARTESIAN_H_
#define CCARTESIAN_H_
#include <iostream>
#include <Eigen/Dense>
#include "CRoboticConstraints.h"
#include <vector>
//#include "/home/guo/Function_Bag/SymbolicC++3-3.35/headers/symbolicc++.h"
//#include <fadbad/fadiff.h>


using namespace std;
using namespace Eigen;

class CCartesian: public CRoboticConstraints {
    public:
        CCartesian();
        virtual ~CCartesian();
        void getRoll(double roll,double x, double y, double z,Matrix4d& rollFrame);
        void getYaw(double yaw,double x, double y, double z,Matrix4d& yawFrame);
        void getPitch(double pitch ,double x,double y,double z,Matrix4d& pitchFrame);
        bool IsInputInLimit(int t1, int t2, int t3, int t4, int t5, int t6);
        Eigen::MatrixXf GetPose(int leg,Eigen::MatrixXf theta);
        Eigen::MatrixXf Jacobian(int leg,Eigen::MatrixXf theta);
    private:
        float t1,t2,t3;
        vector<double> linkLength;
        double th_delt;

        Matrix4d hip_roll_frame;
        Matrix4d hip_yaw_frame;
        Matrix4d hip_pitch_frame;
        Matrix4d knee_pitch_frame;
        Matrix4d ankle_pitch_frame;
        Matrix4d foot_pitch_frame;
        Matrix4d foot_roll_frame;
        Matrix4d trunk_to_foot;

};

#endif