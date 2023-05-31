#include "CCartesian.h"
#include<iostream>
#include <Eigen/Dense>
using namespace std;

CCartesian::CCartesian() {
	// TODO Auto-generated constructor stub

    linkLength.resize(7);
    linkLength[0] = 0.115;
    linkLength[1] = 0.0;
    linkLength[2] = 0.127;
    linkLength[3] = 0.145;
    linkLength[4] = 0.50009;
    linkLength[5] = 0.41507;
    linkLength[6] = 0.0;

    th_delt = 0.04625;

}

CCartesian::~CCartesian() {
	// TODO Auto-generated destructor stub
}

void CCartesian::getRoll(double roll,double x, double y, double z, Matrix4d& rollFrame)
{
    rollFrame << 1, 0, 0, x,
            0, cos(roll),-sin(roll),y,
            0, sin(roll), cos(roll),z,
            0,0,0,1;
}
void CCartesian::getYaw(double yaw,double x, double y, double z,Matrix4d& yawFrame)
{
    yawFrame << cos(yaw),-sin(yaw),0,x,
            sin(yaw), cos(yaw),0,y,
            0,0,1,z,
            0,0,0,1;
}
void CCartesian::getPitch(double pitch ,double x,double y,double z,Matrix4d& pitchFrame)
{
    pitchFrame << cos(pitch),0,sin(pitch),x,
            0,1,0,y,
            -sin(pitch),0, cos(pitch),z,
            0,0,0,1;

}
bool CCartesian::IsInputInLimit(int t1,int t2, int t3, int t4, int t5, int t6){
    if ((-90 <= t1) && (t1 <= 90) && (-90 <= t2) && (t2 <= 90) && (-90 <= t3) && (t3 <= 90) && (-135 <= t4) && (t4 <= 135) && (-135 <= t5) && (t5 <= 135))
    {
        return true;
    }
    else
        return false;
}

Eigen::MatrixXf CCartesian::GetPose(int leg,Eigen::MatrixXf theta)
{
    cout<<"leg = "<<leg<<endl;
    ///get left leg
    if(leg==-1)
    {
        getRoll(theta(0, 0),0,linkLength[0],0,hip_roll_frame);
        ///定义hip_yaw
        getYaw(theta(1, 0),-linkLength[1],0,0,hip_yaw_frame);
        ///定义hip_pitcht
        getPitch(theta(2, 0),0,0,-linkLength[2],hip_pitch_frame);
        ///定义knee pitch
        getPitch(theta(3, 0),0,0,-linkLength[3],knee_pitch_frame);
        ///定义ankle pitch
        getPitch(th_delt - theta(3, 0),0,0,-linkLength[4],ankle_pitch_frame);
        ///定义foot pitch
        getPitch(theta(4, 0),0,0,-linkLength[5],foot_pitch_frame);
        ///定义foot roll
        getRoll(theta(5, 0),0,0,-linkLength[6],foot_roll_frame);
    }
    else
    {
        ///get right leg
        getRoll(theta(0, 0),0,-linkLength[0],0,hip_roll_frame);
        ///定义hip_yaw
        getYaw(theta(1, 0),-linkLength[1],0,0,hip_yaw_frame);
        ///定义hip_pitcht
        getPitch(theta(2, 0),0,0,-linkLength[2],hip_pitch_frame);
        ///定义knee pitch
        getPitch(theta(3, 0),0,0,-linkLength[3],knee_pitch_frame);
        ///定义ankle pitch
        getPitch(th_delt - theta(3, 0),0,0,-linkLength[4],ankle_pitch_frame);
        ///定义foot pitch
        getPitch(theta(4, 0),0,0,-linkLength[5],foot_pitch_frame);
        ///定义foot roll
        getRoll(theta(5, 0),0,0,-linkLength[6],foot_roll_frame);
    }

    trunk_to_foot = hip_roll_frame*hip_yaw_frame*hip_pitch_frame*knee_pitch_frame
                   *ankle_pitch_frame*foot_pitch_frame*foot_roll_frame;

    Eigen::MatrixXf pose(4,4);
    for(int i=0;i<4;i++)
    {
        pose(0,i) = trunk_to_foot(0,i);
        pose(1,i) = trunk_to_foot(1,i);
        pose(2,i) = trunk_to_foot(2,i);
        pose(3,i) = trunk_to_foot(3,i);
    }
    return pose;
}

Eigen::MatrixXf CCartesian::Jacobian(int leg,Eigen::MatrixXf theta)
{
    Eigen::MatrixXf J(6,6);
    double th1,th2,th3,th4,th5,th6,th7;
    double L1,L2,L3,L4,L5,L6,L7;

    th1 = theta(0, 0);
    th2 = theta(1, 0);
    th3 = theta(2, 0);
    th4 = theta(3, 0);
    th5 = -th4 + th_delt;
    th6 = theta(4, 0);
    th7 = theta(5, 0);

    if(leg == 1) //right
    {
        L1 = linkLength[0];
        L2 = linkLength[1];
        L3 = linkLength[2];
        L4 = linkLength[3];
        L5 = linkLength[4];
        L6 = linkLength[5];
        L7 = linkLength[6];
    }
    else //left
    {
        L1 = -linkLength[0];
        L2 = linkLength[1];
        L3 = linkLength[2];
        L4 = linkLength[3];
        L5 = linkLength[4];
        L6 = linkLength[5];
        L7 = linkLength[6];
    }

    ///get right leg
    getRoll(theta(0, 0),0,-leg*linkLength[0],0,hip_roll_frame);
    ///定义hip_yaw
    getYaw(theta(1, 0),-linkLength[1],0,0,hip_yaw_frame);
    ///定义hip_pitcht
    getPitch(theta(2, 0),0,0,-linkLength[2],hip_pitch_frame);
    ///定义knee pitch
    getPitch(theta(3, 0),0,0,-linkLength[3],knee_pitch_frame);
    ///定义ankle pitch
    getPitch(th_delt - theta(3, 0),0,0,-linkLength[4],ankle_pitch_frame);
    ///定义foot pitch
    getPitch(theta(4, 0),0,0,-linkLength[5],foot_pitch_frame);
    ///定义foot roll
    getRoll(theta(5, 0),0,0,-linkLength[6],foot_roll_frame);

    Matrix4d hip_roll_pose   = hip_roll_frame;
    Matrix4d hip_yaw_pose    = hip_roll_frame*hip_yaw_frame;
    Matrix4d hip_pitch_pose  = hip_roll_frame*hip_yaw_frame*hip_pitch_frame;
    Matrix4d knee_pitch_pose = hip_roll_frame*hip_yaw_frame*hip_pitch_frame*knee_pitch_frame;
    Matrix4d foot_pitch_pose = hip_roll_frame*hip_yaw_frame*hip_pitch_frame*knee_pitch_frame*ankle_pitch_frame*foot_pitch_frame;
    Matrix4d foot_roll_pose  = hip_roll_frame*hip_yaw_frame*hip_pitch_frame*knee_pitch_frame*ankle_pitch_frame*foot_pitch_frame*foot_roll_frame;

//    J(0,0) = ;
//    J(0,1) = ;
//    J(0,2) = ;
//    J(0,3) = ;
//    J(0,4) = ;
//    J(0,5) = ;
//
//    J(1,0) = ;
//    J(1,1) = ;
//    J(1,2) = ;
//    J(1,3) = ;
//    J(1,4) = ;
//    J(1,5) = ;
//
//    J(2,0) = ;
//    J(2,1) = ;
//    J(2,2) = ;
//    J(2,3) = ;
//    J(2,4) = ;
//    J(2,5) = ;
//
//    J(3,0) = ;
//    J(3,1) = ;
//    J(3,2) = ;
//    J(3,3) = ;
//    J(3,4) = ;
//    J(3,5) = ;
//
//    J(4,0) = ;
//    J(4,1) = ;
//    J(4,2) = ;
//    J(4,3) = ;
//    J(4,4) = ;
//    J(4,5) = ;
//
//    J(5,0) = ;
//    J(5,1) = ;
//    J(5,2) = ;
//    J(5,3) = ;
//    J(5,4) = ;
//    J(5,5) = ;
    J(0,0) = 0;
    J(0,1) = sin(th2)*(L5*sin(th3 + th4) + L6*sin(th3 + th_delt) + L4*sin(th3) + L7*sin(th3 + th6 + th_delt));
    J(0,2) = -cos(th2)*(L5*cos(th3 + th4) + L6*cos(th3 + th_delt) + L4*cos(th3) + L7*cos(th3 + th6 + th_delt));
    J(0,3) = -L5*cos(th3 + th4)*cos(th2);
    J(0,4) = -L7*(cos(th2 + th3 + th6 + th_delt)/2 + cos(th3 - th2 + th6 + th_delt)/2);
    J(0,5) = 0;

    J(1,0) = L3*cos(th1) + L4*cos(th1)*cos(th3) + L5*cos(th1)*cos(th3)*cos(th4) + L6*cos(th1)*cos(th3)*cos(th_delt) - L5*cos(th1)*sin(th3)*sin(th4) - L6*cos(th1)*sin(th3)*sin(th_delt) + L4*sin(th1)*sin(th2)*sin(th3) + L7*cos(th1)*cos(th3)*cos(th6)*cos(th_delt) - L7*cos(th1)*cos(th3)*sin(th6)*sin(th_delt) - L7*cos(th1)*cos(th6)*sin(th3)*sin(th_delt) - L7*cos(th1)*cos(th_delt)*sin(th3)*sin(th6) + L5*cos(th3)*sin(th1)*sin(th2)*sin(th4) + L5*cos(th4)*sin(th1)*sin(th2)*sin(th3) + L6*cos(th3)*sin(th1)*sin(th2)*sin(th_delt) + L6*cos(th_delt)*sin(th1)*sin(th2)*sin(th3) + L7*cos(th3)*cos(th6)*sin(th1)*sin(th2)*sin(th_delt) + L7*cos(th3)*cos(th_delt)*sin(th1)*sin(th2)*sin(th6) + L7*cos(th6)*cos(th_delt)*sin(th1)*sin(th2)*sin(th3) - L7*sin(th1)*sin(th2)*sin(th3)*sin(th6)*sin(th_delt);
    J(1,1) = -cos(th1)*cos(th2)*(L5*sin(th3 + th4) + L6*sin(th3 + th_delt) + L4*sin(th3) + L7*sin(th3 + th6 + th_delt));
    J(1,2) = L5*cos(th1)*sin(th2)*sin(th3)*sin(th4) - L4*cos(th1)*cos(th3)*sin(th2) - L5*cos(th3)*sin(th1)*sin(th4) - L5*cos(th4)*sin(th1)*sin(th3) - L6*cos(th3)*sin(th1)*sin(th_delt) - L6*cos(th_delt)*sin(th1)*sin(th3) - L5*cos(th1)*cos(th3)*cos(th4)*sin(th2) - L6*cos(th1)*cos(th3)*cos(th_delt)*sin(th2) - L7*cos(th3)*cos(th6)*sin(th1)*sin(th_delt) - L7*cos(th3)*cos(th_delt)*sin(th1)*sin(th6) - L7*cos(th6)*cos(th_delt)*sin(th1)*sin(th3) - L4*sin(th1)*sin(th3) + L6*cos(th1)*sin(th2)*sin(th3)*sin(th_delt) + L7*sin(th1)*sin(th3)*sin(th6)*sin(th_delt) + L7*cos(th1)*cos(th3)*sin(th2)*sin(th6)*sin(th_delt) + L7*cos(th1)*cos(th6)*sin(th2)*sin(th3)*sin(th_delt) + L7*cos(th1)*cos(th_delt)*sin(th2)*sin(th3)*sin(th6) - L7*cos(th1)*cos(th3)*cos(th6)*cos(th_delt)*sin(th2);
    J(1,3) = -L5*(cos(th4)*(sin(th1)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) + sin(th4)*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3)));
    J(1,4) = -L7*(cos(th3)*cos(th6)*sin(th1)*sin(th_delt) + cos(th3)*cos(th_delt)*sin(th1)*sin(th6) + cos(th6)*cos(th_delt)*sin(th1)*sin(th3) - sin(th1)*sin(th3)*sin(th6)*sin(th_delt) + cos(th1)*cos(th3)*cos(th6)*cos(th_delt)*sin(th2) - cos(th1)*cos(th3)*sin(th2)*sin(th6)*sin(th_delt) - cos(th1)*cos(th6)*sin(th2)*sin(th3)*sin(th_delt) - cos(th1)*cos(th_delt)*sin(th2)*sin(th3)*sin(th6));
    J(1,5) = 0;

    J(2,0) = L3*sin(th1) + L4*cos(th3)*sin(th1) + L5*cos(th3)*cos(th4)*sin(th1) + L6*cos(th3)*cos(th_delt)*sin(th1) - L4*cos(th1)*sin(th2)*sin(th3) - L5*sin(th1)*sin(th3)*sin(th4) - L6*sin(th1)*sin(th3)*sin(th_delt) + L7*cos(th3)*cos(th6)*cos(th_delt)*sin(th1) - L5*cos(th1)*cos(th3)*sin(th2)*sin(th4) - L5*cos(th1)*cos(th4)*sin(th2)*sin(th3) - L6*cos(th1)*cos(th3)*sin(th2)*sin(th_delt) - L6*cos(th1)*cos(th_delt)*sin(th2)*sin(th3) - L7*cos(th3)*sin(th1)*sin(th6)*sin(th_delt) - L7*cos(th6)*sin(th1)*sin(th3)*sin(th_delt) - L7*cos(th_delt)*sin(th1)*sin(th3)*sin(th6) - L7*cos(th1)*cos(th3)*cos(th6)*sin(th2)*sin(th_delt) - L7*cos(th1)*cos(th3)*cos(th_delt)*sin(th2)*sin(th6) - L7*cos(th1)*cos(th6)*cos(th_delt)*sin(th2)*sin(th3) + L7*cos(th1)*sin(th2)*sin(th3)*sin(th6)*sin(th_delt);
    J(2,1) = -cos(th2)*sin(th1)*(L5*sin(th3 + th4) + L6*sin(th3 + th_delt) + L4*sin(th3) + L7*sin(th3 + th6 + th_delt));
    J(2,2) = L4*cos(th1)*sin(th3) + L5*cos(th1)*cos(th3)*sin(th4) + L5*cos(th1)*cos(th4)*sin(th3) + L6*cos(th1)*cos(th3)*sin(th_delt) + L6*cos(th1)*cos(th_delt)*sin(th3) - L4*cos(th3)*sin(th1)*sin(th2) + L7*cos(th1)*cos(th3)*cos(th6)*sin(th_delt) + L7*cos(th1)*cos(th3)*cos(th_delt)*sin(th6) + L7*cos(th1)*cos(th6)*cos(th_delt)*sin(th3) - L5*cos(th3)*cos(th4)*sin(th1)*sin(th2) - L6*cos(th3)*cos(th_delt)*sin(th1)*sin(th2) - L7*cos(th1)*sin(th3)*sin(th6)*sin(th_delt) + L5*sin(th1)*sin(th2)*sin(th3)*sin(th4) + L6*sin(th1)*sin(th2)*sin(th3)*sin(th_delt) - L7*cos(th3)*cos(th6)*cos(th_delt)*sin(th1)*sin(th2) + L7*cos(th3)*sin(th1)*sin(th2)*sin(th6)*sin(th_delt) + L7*cos(th6)*sin(th1)*sin(th2)*sin(th3)*sin(th_delt) + L7*cos(th_delt)*sin(th1)*sin(th2)*sin(th3)*sin(th6);
    J(2,3) = L5*(cos(th4)*(cos(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2)) + sin(th4)*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3)));
    J(2,4) = L7*(cos(th1)*cos(th3)*cos(th6)*sin(th_delt) + cos(th1)*cos(th3)*cos(th_delt)*sin(th6) + cos(th1)*cos(th6)*cos(th_delt)*sin(th3) - cos(th1)*sin(th3)*sin(th6)*sin(th_delt) - cos(th3)*cos(th6)*cos(th_delt)*sin(th1)*sin(th2) + cos(th3)*sin(th1)*sin(th2)*sin(th6)*sin(th_delt) + cos(th6)*sin(th1)*sin(th2)*sin(th3)*sin(th_delt) + cos(th_delt)*sin(th1)*sin(th2)*sin(th3)*sin(th6));
    J(2,5) = 0;

    J(3,0) = hip_roll_pose(0,0);
    J(3,1) = hip_yaw_pose(0,2);
    J(3,2) = hip_pitch_pose(0,1);
    J(3,3) = knee_pitch_pose(0,1);
    J(3,4) = foot_pitch_pose(0,1);
    J(3,5) = foot_roll_pose(0,0);

    J(4,0) = hip_roll_pose(1,0);
    J(4,1) = hip_yaw_pose(1,2);
    J(4,2) = hip_pitch_pose(1,1);
    J(4,3) = knee_pitch_pose(1,1);
    J(4,4) = foot_pitch_pose(1,1);
    J(4,5) = foot_roll_pose(1,0);

    J(5,0) = hip_roll_pose(2,0);
    J(5,1) = hip_yaw_pose(2,2);
    J(5,2) = hip_pitch_pose(2,1);
    J(5,3) = knee_pitch_pose(2,1);
    J(5,4) = foot_pitch_pose(2,1);
    J(5,5) = foot_roll_pose(2,0);

    return J;
}