#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#define PI (3.1415926535897932346f)

using namespace std;
extern "C" {

void leg_ik(double leg_lenth[], double end_pos[], double motor_way[], double st_leg_angle[], bool LorR) {
    /* 
    input:
        leg_lenth :: lap leg foot ankle(d L1 h1 h2)
        end_pos :: x(front) y(left) z(up) roll pitch yaw
    output:
        st_leg_angle :: joint angle , theta 0 - 5 in ID => model.png
    */
    Eigen::Matrix3d R_t;
    R_t = Eigen::AngleAxisd(end_pos[5], Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd(end_pos[4], Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd(end_pos[3], Eigen::Vector3d::UnitX());
    
    Eigen::Vector3d d_t(end_pos[0], end_pos[1], end_pos[2]);
    
    //caculate self.theta 3 1 0 (HT_ID: 14 12 11)
    Eigen::Matrix3d R_ti = R_t.transpose();
    Eigen::Vector3d d_3(0.0,0.0,leg_lenth[2]);
    Eigen::Vector3d da = - R_ti * d_t - d_3;
    double la = da.norm();
        
    // 3
    double theta_a = 0;
    if(abs(leg_lenth[0] - leg_lenth[1]) > la){
        cout<<"error - too close"<<endl;
        st_leg_angle[3] = - PI; 
        theta_a = PI;}
    else if((leg_lenth[0] + leg_lenth[1]) > la){
        st_leg_angle[3] = acos( ( pow(leg_lenth[0],2)  + pow(leg_lenth[1],2) - pow(la,2) )  / (2 * leg_lenth[0] * leg_lenth[1]) )- PI;
        theta_a  = acos( (pow(leg_lenth[1],2)  + pow(la,2) - pow(leg_lenth[0],2) ) / (2 * leg_lenth[1] * la) );}
    else{
        cout<<"error - too far"<<endl;
        st_leg_angle[3] = 0.0;
        theta_a = 0.0;}

    // 1 - Y
    st_leg_angle[1] = theta_a + asin(da[0] / la);
        
    // 0 - X
    st_leg_angle[0] = atan(- da[1] / (da[2] - leg_lenth[2]));

    // caculate self.theta  2 4 5
    Eigen::Matrix3d R_013;
    R_013 = Eigen::AngleAxisd(st_leg_angle[0], Eigen::Vector3d::UnitX()) * 
            Eigen::AngleAxisd(st_leg_angle[1] + st_leg_angle[3], Eigen::Vector3d::UnitY());
    Eigen::Matrix3d Rlap = R_013.transpose() * R_ti;
    
    //ZXY EULAR (YXZ Bryan)  
    double err = 0.001;
    Eigen::Vector2d v_x(Rlap(1,0), Rlap(1,1));
    double ox = atan2(-Rlap(1,2), v_x.norm());

    if (ox >= PI/2 - err && ox <= PI/2 + err){
       st_leg_angle[5] = atan2(Rlap(2,0), Rlap(0,0));
       st_leg_angle[4] = PI/2;
       st_leg_angle[2] = 0.0;
       }
    else if (ox >= -(PI/2) - err && ox <= -(PI/2) + err){
       st_leg_angle[5] = atan2(-Rlap(2,0), -Rlap(0,0));
       st_leg_angle[4] = -PI/2;
       st_leg_angle[2] = 0.0;
       }
    else{
       st_leg_angle[5] = atan2(Rlap(1,0)/cos(ox), Rlap(1,1)/cos(ox));
       st_leg_angle[4] = ox;
       st_leg_angle[2] = atan2(Rlap(0,2)/cos(ox), Rlap(2,2)/cos(ox));
       }
        
    // parallel ankle caculate reference -> git@github.com:rocketman123456/ros2_ws.git
    double ty = st_leg_angle[1];
    double tx = st_leg_angle[0];
    double d =  leg_lenth[3];
    double L1 = leg_lenth[4];
    double h1 = leg_lenth[5];
    double h2 = leg_lenth[6]; 
    if (LorR){
    h1 = leg_lenth[6];
    h2 = leg_lenth[5];
    tx = -st_leg_angle[0];
    } 
    double cx = cos(tx);
    double sx = sin(tx);
    double cy = cos(ty);
    double sy = sin(ty);

    double AL = - L1 * L1 * cy + L1 * d * sx * sy;
    double BL = - L1 * L1 * sy + L1 * h1 - L1 * d * sx * cy;
    double CL = -(L1 * L1 + d * d - d * d *cx - L1 * h1 * sy - d * h1 * sx * cy);

    double LenL = sqrt(AL * AL + BL * BL);

    double AR = - L1 * L1 * cy - L1 * d * sx * sy;
    double BR = - L1 * L1 * sy + L1 * h2 + L1 * d * sx * cy;
    double CR = -(L1 * L1 + d * d - d * d *cx - L1 * h2 * sy + d * h2 * sx * cy);

    double LenR = sqrt(AR * AR + BR * BR);

    if (LenL <= abs(CL) || LenR <= abs(CR))
    {
        cout<<"error ankle"<<endl;
        st_leg_angle[1] = 0;
        st_leg_angle[0] = 0;
    }
    else
    {
        st_leg_angle[1] = asin(CL / LenL) - asin(AL / LenL);
        st_leg_angle[0] = asin(CR / LenR) - asin(AR / LenR);
    }
    
    // parallel lap caculate
    st_leg_angle[3] = st_leg_angle[3] + st_leg_angle[2];
    
    // motor way
    for(int i = 0; i < 6; i++){
       st_leg_angle[i] *= motor_way[i];
    };

}

}
