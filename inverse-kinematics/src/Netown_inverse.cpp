#include "Netown_inverse.h"

Netown_inverse::Netown_inverse() {

    linkLength.resize(7);
    linkLength[0] = 0.115;
    linkLength[1] = 0.0;
    linkLength[2] = 0.127;
    linkLength[3] = 0.145;
    linkLength[4] = 0.50009;
    linkLength[5] = 0.41507;
    linkLength[6] = 0.0;

    th_delt = 0.04625;

    e_pos.resize(3,1);
    pos_d.resize(3, 1); // Input cartesian x, y, z coordinates from the user
    e_pose.resize(3, 3);
    delt_R.resize(3, 3);
    ln_Rv.resize(3,1);
    e.resize(6,1);
    err.resize(6,1);

    i_1_theta.resize(6, 1);  // temporary joint angles variables in every iteration
    i_theta.resize(6, 1);    // final joint angles
    i_theta_Left.resize(6, 1);
    i_theta_Right.resize(6, 1);
    pose_d.resize(3,3);

    X_roll.resize(3,3);
    Y_pitch.resize(3,3);
    Z_yaw.resize(3,3);
    ZYX.resize(3,1);

    i_theta_Right(0, 0) = 0;                // Initial guess
    i_theta_Right(1, 0) = 0;
    i_theta_Right(2, 0) = -0.712;
    i_theta_Right(3, 0) = 1.462;
    i_theta_Right(4, 0) = 0.666;
    i_theta_Right(5, 0) = 0;

    i_theta_Left(0, 0) = 0;                // Initial guess
    i_theta_Left(1, 0) = 0;
    i_theta_Left(2, 0) = -0.712;
    i_theta_Left(3, 0) = 1.462;
    i_theta_Left(4, 0) = 0.666;
    i_theta_Left(5, 0) = 0;

    hiproll_offset    =  0;
    hipyaw_offset     =  0;
    hippitch_offset   =  + 0.71192;
    knee_offset       =  - 1.46224;
    footpitch_offset  = - 0.66637;
    footroll_offset  =  0.0;
    anklepitch_offset =  1.41668 ;

}

Netown_inverse::~Netown_inverse() {
    // TODO Auto-generated destructor stub
}

void Netown_inverse::get_Angle(double* tChest, double* tAnkle, vector<double> &q, int whleg)
{
    cout<<"whleg = "<<whleg<<endl;
    ///init theta
    if(whleg == 1)
    {
        i_theta= i_theta_Right;
    }
    else
    {
        i_theta= i_theta_Left;
    }
    ///get pose
    pos_d(0,0) = tAnkle[0] - tChest[0];
    pos_d(1,0) = tAnkle[1] - tChest[1];
    pos_d(2,0) = tAnkle[2] - tChest[2];
    ///get pose Matrix
    ZYX(0,0) = tAnkle[3]/180.0*M_PI;  //x
    ZYX(1,0) = tAnkle[4]/180.0*M_PI;  //y
    ZYX(2,0) = tAnkle[5]/180.0*M_PI;  //z
    X_roll << 1,  0,  0,
            0, cos(ZYX(0,0)),-sin(ZYX(0,0)),
            0, sin(ZYX(0,0)), cos(ZYX(0,0));
    Y_pitch << cos(ZYX(1,0)),0,sin(ZYX(1,0)),
            0,  1,  0,
            -sin(ZYX(1,0)),0, cos(ZYX(1,0));
    Z_yaw << cos(ZYX(2,0)),-sin(ZYX(2,0)),0,
            sin(ZYX(2,0)), cos(ZYX(2,0)),0,
            0,  0,  1;

    pose_d = Z_yaw*Y_pitch*X_roll;
    //cout<<"pose_d = "<<pose_d<<endl;

    pose_get.resize(3,3);
    pose_get_transpose.resize(3,3);
    pose.resize(4,4);        // temporary pose variables in every iteration
    J.resize(6,6);           // Jacobian

    pose = cu.GetPose(whleg,i_theta);       // calculate pose for initial guess
    cout<<"pose = "<<pose<<endl;
    ///get pose
    pose_get(0,0) = pose(0,0);
    pose_get(0,1) = pose(0,1);
    pose_get(0,2) = pose(0,2);
    pose_get(1,0) = pose(1,0);
    pose_get(1,1) = pose(1,1);
    pose_get(1,2) = pose(1,2);
    pose_get(2,0) = pose(2,0);
    pose_get(2,1) = pose(2,1);
    pose_get(2,2) = pose(2,2);
    ///get pose transpose
    pose_get_transpose = pose_get.transpose();
    delt_R = pose_get_transpose*pose_d;

    theta_ = acos((delt_R(0,0)+delt_R(1,1)+delt_R(2,2)-1)/2);
    if(theta_ == 0)
    {
        //cout<<"delt_R == eye4"<<endl;
        ln_Rv(0,0) = 0;
        ln_Rv(1,0) = 0;
        ln_Rv(2,0) = 0;
    }
    else
    {
        //cout<<"delt_R != eye4"<<endl;
        /// get pose err // is delt w

        ln_Rv(0,0) = theta_/(2*sin(theta_))*(delt_R(2,1) - delt_R(1,2));
        ln_Rv(1,0) = theta_/(2*sin(theta_))*(delt_R(0,2) - delt_R(2,0));
        ln_Rv(2,0) = theta_/(2*sin(theta_))*(delt_R(1,0) - delt_R(0,1));

    }
    ///get pos err
    e_pos(0,0) = pos_d(0,0) - pose(0,3);                    // error between current and input pose
    e_pos(1,0) = pos_d(1,0) - pose(1,3);
    e_pos(2,0) = pos_d(2,0) - pose(2,3);
    ///get 2 err
    //err = ln_Rv.norm() + e_pos.norm();
    err(0,0) = e_pos(0,0);
    err(1,0) = e_pos(1,0);
    err(2,0) = e_pos(2,0);
    err(3,0) = ln_Rv(0,0);
    err(4,0) = ln_Rv(1,0);
    err(5,0) = ln_Rv(2,0);

    e << e_pos(0,0),e_pos(1,0),e_pos(2,0),
            ln_Rv(0,0),ln_Rv(1,0),ln_Rv(2,0);

    while (abs(err(0,0)) > 0.001 ||  abs(err(1,0)) > 0.001 || abs(err(2,0)) > 0.001 || abs(err(3,0)) > 0.001 || abs(err(4,0)) > 0.001 || abs(err(5,0)) > 0.001)
    {
        J = cu.Jacobian(whleg,i_theta);     // find jacobian for current joint angles

        Eigen::MatrixXf JPsedoInv = J.completeOrthogonalDecomposition().pseudoInverse();  // find pseudo inverse of Jacobian
        i_1_theta = i_theta + JPsedoInv * e;   // get new joint angles by compensating the error

        pose = cu.GetPose(whleg,i_1_theta);   // calculate pose for current angles
        //cout<<"pose = "<<pose<<endl;
        ///get pose
        pose_get(0,0) = pose(0,0);
        pose_get(0,1) = pose(0,1);
        pose_get(0,2) = pose(0,2);
        pose_get(1,0) = pose(1,0);
        pose_get(1,1) = pose(1,1);
        pose_get(1,2) = pose(1,2);
        pose_get(2,0) = pose(2,0);
        pose_get(2,1) = pose(2,1);
        pose_get(2,2) = pose(2,2);
        ///get pose transpose
        pose_get_transpose = pose_get.transpose();
        delt_R = pose_get_transpose*pose_d;
        //cout<<"delt_R = "<<delt_R<<endl;

        theta_ = acos((delt_R(0,0)+delt_R(1,1)+delt_R(2,2)-1)/2);
        if(theta_ == 0)
        {
            //cout<<"delt_R == eye4"<<endl;
            ln_Rv(0,0) = 0;
            ln_Rv(1,0) = 0;
            ln_Rv(2,0) = 0;
        }
        else
        {
            //cout<<"delt_R != eye4"<<endl;
            /// get pose err // is delt w

            ln_Rv(0,0) = theta_/(2*sin(theta_))*(delt_R(2,1) - delt_R(1,2));
            ln_Rv(1,0) = theta_/(2*sin(theta_))*(delt_R(0,2) - delt_R(2,0));
            ln_Rv(2,0) = theta_/(2*sin(theta_))*(delt_R(1,0) - delt_R(0,1));
//            cout<<"theta_ = "<<theta_<<endl;
//            cout<<"ln_Rv 1 = "<<(2*sin(theta_))*(delt_R(2,1) - delt_R(1,2))<<endl;
//            cout<<"ln_Rv 2 = "<<(2*sin(theta_))*(delt_R(0,2) - delt_R(2,0))<<endl;
//            cout<<"ln_Rv 3 = "<<(2*sin(theta_))*(delt_R(1,0) - delt_R(0,1))<<endl;
        }
        ///get pos err
        e_pos(0,0) = pos_d(0,0) - pose(0,3);                    // error between current and input pose
        e_pos(1,0) = pos_d(1,0) - pose(1,3);
        e_pos(2,0) = pos_d(2,0) - pose(2,3);
        ///get 2 err
        //err = ln_Rv.norm() + e_pos.norm();
        err(0,0) = e_pos(0,0);
        err(1,0) = e_pos(1,0);
        err(2,0) = e_pos(2,0);
        err(3,0) = ln_Rv(0,0);
        err(4,0) = ln_Rv(1,0);
        err(5,0) = ln_Rv(2,0);
        //cout<<"err = "<<err<<endl;

        /// new error
        e << e_pos(0,0),e_pos(1,0),e_pos(2,0),
                ln_Rv(0,0),ln_Rv(1,0),ln_Rv(2,0);

        //cout<<"e = "<<e<<endl;
        i_theta = i_1_theta;            // new joint angles
    }

    //      Solutions do converge but they are often outside joint limits.                      //
    //      The below method solves this problem to some extent but is not completely correct   //

    for (int i = 0; i < 6; i++)
    {    // process the final joint angles such that they are within specified limits
        if (i_theta(i,0) > 0)
            while (i_theta(i,0) > M_PI)
                i_theta(i,0) = i_theta(i,0) - M_PI;
        else if (i_theta(i,0) < 0)
            while (i_theta(i,0) < -M_PI)
                i_theta(i,0) = M_PI + i_theta(i,0);
        //i_theta(i,0) = i_theta(i,0)*float(180/M_PI);
    }
//    if (cart.IsInputInLimit(int(i_theta(0,0)),int(i_theta(1,0)),int(i_theta(2,0)),int(i_theta(3,0)),int(i_theta(4,0)),int(i_theta(5,0))))  // Are the joint values within specified limits?!
//        cout << "Solution reachable:" << endl;
//    else
//        cout << "solution unreachable:" << endl;
    if(whleg == 1)
    {
        i_theta_Right = i_theta;
    }
    else
    {
        i_theta_Left = i_theta;
    }
    //cout << i_theta << endl;
    q[0] = i_theta(0,0) + hiproll_offset; //hip roll
    q[1] = i_theta(1,0) + hipyaw_offset; //hip yaw
    q[2] = i_theta(2,0) + hippitch_offset; //hip pitch
    q[3] = i_theta(3,0) + knee_offset; //knee
    q[4] = th_delt - i_theta(3,0) + anklepitch_offset; //ankle
    q[5] = i_theta(4,0) + footpitch_offset; //foot pitch
    q[6] = i_theta(5,0) + footroll_offset; //foot roll
    cout<<"i_theta = "<<i_theta<<endl;

}

int main()
{
    Netown_inverse netown_inverse;
    double H_cm[6];
    double H_ra[6];
    double H_la[6];

    vector<double> rq,lq;
    rq.resize(7);
    lq.resize(7);
    ///com
    H_cm[0]=0;H_cm[1]=0;H_cm[2]=0;
    H_cm[3]=0;H_cm[4]=0;H_cm[5]=0;
    ///right foot
    H_ra[0]=0.0103;
    H_ra[1]=-0.115;
    H_ra[2]=-0.92887;
    H_ra[3]=0;
    H_ra[4]=0;
    H_ra[5]=5;
    ///left foot
    H_la[0]=0.0103;
    H_la[1]=0.115;
    H_la[2]=-0.92887;
    H_la[3]=0;
    H_la[4]=0;
    H_la[5]=-5;

    netown_inverse.get_Angle(H_cm, H_ra, rq, 1);
    netown_inverse.get_Angle(H_cm, H_la, lq, -1);
    //std::cout << "rq "<< std::endl;

}