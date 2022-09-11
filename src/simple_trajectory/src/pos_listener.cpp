#include <ros/ros.h>
#include <ros/package.h>
#include <ros/topic.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "control_msgs/JointControllerState.h"
#include <iostream>
#include <math.h>
#include <complex.h>
#include <chrono>
#include <thread>
#include <tuple>

using namespace std;
using namespace Eigen;

#define N_JOINT 6
#define BASE 0
#define SHOULDER 1
#define ELBOW 2
#define WRIST1 3
#define WRIST2 4
#define WRIST3 5
#define PI 3.14159265359
#define INITIALIZE_CYCLE 40
#define PREF_SOLUTION 1
// set to 0 for assignment 1, 2 and 3
#define ASSIGNMENT4 1

const double a[N_JOINT] = {0, -0.425, -0.3922, 0, 0, 0}; //Vector of the a distance (expressed in metres)
const double d[N_JOINT] = {0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.235}; //Vector of the d distance (expressed in metres)
double r = 0;

struct Brick
{
    double x;
    int count;
    double height;  //  only the base (parallelepiped), i.e. without the round parts (technically stud/knob)
    double width;   // short side, "X" in the type name
    double length;  // long side, "Y" in the type name
};
map<string, Brick> stands = {
    {"X1-Y1-Z2", {0.5, 0, 0.0375, 0.031, 0.031}},
    {"X1-Y2-Z1", {0.4, 0, 0.0185, 0.031, 0.063}},
    {"X1-Y2-Z2", {0.3, 0, 0.0375, 0.031, 0.063}},
    {"X1-Y2-Z2-C", {0.2, 0, 0.0375, 0.031, 0.063}},
    {"X1-Y2-Z2-T", {0.1, 0, 0.0375, 0.031, 0.063}},
    {"X1-Y3-Z2", {0.0, 0, 0.0375, 0.031, 0.095}},
    {"X1-Y3-Z2-F", {-0.1, 0, 0.0375, 0.031, 0.095}},
    {"X1-Y4-Z1", {-0.2, 0, 0.0185, 0.031, 0.127}},
    {"X1-Y4-Z2", {-0.3, 0, 0.0375, 0.031, 0.127}},
    {"X2-Y2-Z2", {-0.4, 0, 0.0375, 0.063, 0.063}},
    {"X2-Y2-Z2-F", {-0.5, 0, 0.0375, 0.063, 0.063}},
};
// only for assignment 4
int ind = 0;
/*vector<tuple<string, double, double, double>> ass4 = {
    {"X2-Y2-Z2", 0.0, 0.5, 0.0},
    {"X2-Y2-Z2", 0.0, 0.5, 0.0375},
    {"X2-Y2-Z2-F", 0.0, 0.5, 0.0750}
};*/
vector<tuple<string, double, double, double>> ass4 = {
    {"X1-Y2-Z2", 0.0, 0.44, 0.0},
    {"X1-Y2-Z2", 0.0, 0.5, 0.0},
    {"X1-Y4-Z2", 0.0, 0.47, 0.0375}
};
//------------------------------------------------------------------------------------------------------------------------------
//VETTORE POSIZIONI JOINT
//------------------------------------------------------------------------------------------------------------------------------
double position[N_JOINT];
//------------------------------------------------------------------------------------------------------------------------------
//FINE VETTORE POSIZIONI JOINT
//------------------------------------------------------------------------------------------------------------------------------


typedef struct pos_rot{ //cartesian position and rotation matrix
    Vector3d pos;
    Matrix3d rot;
    pos_rot(Vector3d p, Matrix3d r){
        pos = p; rot = r;
    }
    void stampa(){
        cout << endl;
        cout << "Pos: " <<  endl;
        cout << pos << endl;
        cout << "Rot: " << endl;
        cout << rot << endl;
        cout << endl;
    }
}pos_rot;

pos_rot changeZ(pos_rot p, double z){
    p.pos(2) = z;
    return p;
}

void const stampaMatrice4d(Matrix4d mat){
    cout << endl;
    cout << mat << endl;
    cout << endl;
}

VectorXd selectSolution(MatrixXd m, int sol){ //Seleziona la soluzione da inverse kinematics
    return m.row(sol);
}

bool isNanVectorXd(VectorXd v){ //Controlla se il vettore contiene un nan
    bool ret = false;
    for(int i=0; i < v.size(); i++){
        if(isnan(v(i))){
            ret = true;
        }
    }
    return ret;
}

VectorXd firstSolutionPossible(MatrixXd m){ //Seleziona la prima soluzione non contentente un nan
    for(int i=PREF_SOLUTION; i < 8; i++){
        if(isNanVectorXd(selectSolution(m,i)) == false){
            return selectSolution(m,i);
        }
    }
    if(isNanVectorXd(selectSolution(m,0)) == true){
        cout << endl;
        cout << "ERROR: POSIZIONE IRRANGGIUNGIBILE" << endl;
        cout << endl; 
    }
    return selectSolution(m,0);
}


//------------------------------------------------------------------------------------------------------------------------------
//DIRECT KINEMATICS
//------------------------------------------------------------------------------------------------------------------------------
pos_rot direct(double th[N_JOINT]){
    auto T10f = [th] () {
       Matrix4d ret;
       ret <<   cos(th[BASE]), -sin(th[BASE]), 0, 0,
                sin(th[BASE]), cos(th[BASE]), 0, 0,
                0, 0, 1, d[0],
                0, 0, 0, 1; 
        return ret;
    };
    auto T21f = [th] () {
       Matrix4d ret;
       ret <<   cos(th[SHOULDER]), -sin(th[SHOULDER]), 0, 0,
                0, 0, -1, 0,
                sin(th[SHOULDER]), cos(th[SHOULDER]), 0, 0,
                0, 0, 0, 1;
       return ret; 
    };
    auto T32f = [th] () {
       Matrix4d ret;
       ret <<   cos(th[ELBOW]), -sin(th[ELBOW]), 0, a[1],
                sin(th[ELBOW]), cos(th[ELBOW]), 0, 0,
                0, 0, 1, d[2],
                0, 0, 0, 1;
       return ret; 
    };
    auto T43f = [th] () {
       Matrix4d ret;
       ret <<   cos(th[WRIST1]), -sin(th[WRIST1]), 0, a[2],
                sin(th[WRIST1]), cos(th[WRIST1]), 0, 0,
                0, 0, 1, d[3],
                0, 0, 0, 1;
       return ret; 
    };
    auto T54f = [th] () {
       Matrix4d ret;
       ret <<   cos(th[WRIST2]), -sin(th[WRIST2]), 0, 0,
                0, 0, -1, -d[4],
                sin(th[WRIST2]), cos(th[WRIST2]), 0, 0,
                0, 0, 0, 1;
       return ret; 
    };
    auto T65f = [th] () {
       Matrix4d ret;
       ret <<   cos(th[WRIST3]), -sin(th[WRIST3]), 0, 0,
                0, 0, 1, d[5],
                -sin(th[WRIST3]), -cos(th[WRIST3]), 0, 0,
                0, 0, 0, 1;
       return ret; 
    };
    Matrix4d T10m = T10f();
    Matrix4d T21m = T21f();
    Matrix4d T32m = T32f();
    Matrix4d T43m = T43f();
    Matrix4d T54m = T54f();
    Matrix4d T65m = T65f();
    Matrix4d T06 = T10m*T21m*T32m*T43m*T54m*T65m;
    pos_rot pr1(Vector3d(T06(0,3),T06(1,3),T06(2,3)),T06.topLeftCorner(3,3));
    //stampaMatrice4d(T06);
    return pr1;
}
//------------------------------------------------------------------------------------------------------------------------------
//FINE DIRECT KINEMATICS
//------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------
//INVERSE KINEMATICS
//------------------------------------------------------------------------------------------------------------------------------
MatrixXd inverse(pos_rot pr){
    Matrix4d T60;
    T60.topLeftCorner(3,3) = pr.rot;
    T60.topRightCorner(3,1) = pr.pos;
    T60.bottomLeftCorner(1,3) = MatrixXd::Zero(1,3);
    T60(3,3) = 1;

    auto T10f = [] (double th) {
       Matrix4d ret;
       ret <<   cos(th), -sin(th), 0, 0,
                sin(th), cos(th), 0, 0,
                0, 0, 1, d[0],
                0, 0, 0, 1; 
        return ret;
    };
    auto T21f = [] (double th) {
       Matrix4d ret;
       ret <<   cos(th), -sin(th), 0, 0,
                0, 0, -1, 0,
                sin(th), cos(th), 0, 0,
                0, 0, 0, 1;
       return ret; 
    };
    auto T32f = [] (double th) {
       Matrix4d ret;
       ret <<   cos(th), -sin(th), 0, a[1],
                sin(th), cos(th), 0, 0,
                0, 0, 1, d[2],
                0, 0, 0, 1;
       return ret; 
    };
    auto T43f = [] (double th) {
       Matrix4d ret;
       ret <<   cos(th), -sin(th), 0, a[2],
                sin(th), cos(th), 0, 0,
                0, 0, 1, d[3],
                0, 0, 0, 1;
       return ret; 
    };
    auto T54f = [] (double th) {
       Matrix4d ret;
       ret <<   cos(th), -sin(th), 0, 0,
                0, 0, -1, -d[4],
                sin(th), cos(th), 0, 0,
                0, 0, 0, 1;
       return ret; 
    };
    auto T65f = [] (double th) {
       Matrix4d ret;
       ret <<   cos(th), -sin(th), 0, 0,
                0, 0, 1, d[5],
                -sin(th), -cos(th), 0, 0,
                0, 0, 0, 1;
       return ret; 
    };

    //FINDING TH1
    Vector4d temp(0,0,-d[5],1);
    Vector4d p50 = T60*temp;
    double th1_1 = std::real(atan2(p50(1), p50(0)) + acos(d[3]/hypot(p50(1), p50(0))))+PI/2;
    double th1_2 = std::real(atan2(p50(1), p50(0)) - acos(d[3]/hypot(p50(1), p50(0))))+PI/2;

    //FINDING TH5
    double th5_1 = +std::real(acos((pr.pos(0)*sin(th1_1) - pr.pos(1)*cos(th1_1)-d[3]) / d[5]));
    double th5_2 = -std::real(acos((pr.pos(0)*sin(th1_1) - pr.pos(1)*cos(th1_1)-d[3]) / d[5]));
    double th5_3 = +std::real(acos((pr.pos(0)*sin(th1_2) - pr.pos(1)*cos(th1_2)-d[3]) / d[5]));
    double th5_4 = -std::real(acos((pr.pos(0)*sin(th1_2) - pr.pos(1)*cos(th1_2)-d[3]) / d[5]));

    //FINDING TH6
    Matrix4d T06 = T60.inverse();
    Vector3d Xhat(T06(0,0),T06(1,0),T06(2,0));
    Vector3d Yhat(T06(0,1),T06(1,1),T06(2,1));
    //related to th11 a th51
    double th6_1 = std::real(atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_1), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_1)));
    //related to th11 a th52
    double th6_2 = std::real(atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1))/sin(th5_2)), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1))/sin(th5_2))));
    //related to th12 a th53
    double th6_3 = std::real(atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2))/sin(th5_3)), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2))/sin(th5_3))));
    //related to th12 a th54
    double th6_4 = std::real(atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2))/sin(th5_4)), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2))/sin(th5_4))));

    //FINDING TH3
    Matrix4d T41m = T10f(th1_1).inverse()*T60*T65f(th6_1).inverse()*T54f(th5_1).inverse();
    Vector3d p41_1(T41m(0,3),T41m(1,3),T41m(2,3));
    double p41xz_1 = hypot(p41_1(0), p41_1(2));

    T41m = T10f(th1_1).inverse()*T60*T65f(th6_2).inverse()*T54f(th5_2).inverse();
    Vector3d p41_2(T41m(0,3),T41m(1,3),T41m(2,3));
    double p41xz_2 = hypot(p41_2(0), p41_2(2));
 
    T41m = T10f(th1_2).inverse()*T60*T65f(th6_3).inverse()*T54f(th5_3).inverse();
    Vector3d p41_3(T41m(0,3),T41m(1,3),T41m(2,3));
    double p41xz_3 = hypot(p41_3(0), p41_3(2));
 
    T41m = T10f(th1_2).inverse()*T60*T65f(th6_4).inverse()*T54f(th5_4).inverse();
    Vector3d p41_4(T41m(0,3),T41m(1,3),T41m(2,3));
    double p41xz_4 = hypot(p41_4(0), p41_4(2));

    double th3_1 = std::real(acos((pow(p41xz_1,2)-pow(a[1],2)-pow(a[2],2))/(2*a[1]*a[2])));
    double th3_2 = std::real(acos((pow(p41xz_2,2)-pow(a[1],2)-pow(a[2],2))/(2*a[1]*a[2])));
    double th3_3 = std::real(acos((pow(p41xz_3,2)-pow(a[1],2)-pow(a[2],2))/(2*a[1]*a[2])));
    double th3_4 = std::real(acos((pow(p41xz_4,2)-pow(a[1],2)-pow(a[2],2))/(2*a[1]*a[2])));
    double th3_5 = -th3_1;
    double th3_6 = -th3_2;
    double th3_7 = -th3_3;
    double th3_8 = -th3_4;

    //FINDING TH2
    double th2_1 = std::real(atan2(-p41_1(2), -p41_1(0))-asin((-a[2]*sin(th3_1))/p41xz_1));
    double th2_2 = std::real(atan2(-p41_2(2), -p41_2(0))-asin((-a[2]*sin(th3_2))/p41xz_2));
    double th2_3 = std::real(atan2(-p41_3(2), -p41_3(0))-asin((-a[2]*sin(th3_3))/p41xz_3));
    double th2_4 = std::real(atan2(-p41_4(2), -p41_4(0))-asin((-a[2]*sin(th3_4))/p41xz_4));
    double th2_5 = std::real(atan2(-p41_1(2), -p41_1(0))-asin((a[2]*sin(th3_1))/p41xz_1));
    double th2_6 = std::real(atan2(-p41_2(2), -p41_2(0))-asin((a[2]*sin(th3_2))/p41xz_2));
    double th2_7 = std::real(atan2(-p41_3(2), -p41_3(0))-asin((a[2]*sin(th3_3))/p41xz_3));
    double th2_8 = std::real(atan2(-p41_4(2), -p41_4(0))-asin((a[2]*sin(th3_4))/p41xz_4));

    //FINDING TH4
    Matrix4d T43m = T32f(th3_1).inverse()*T21f(th2_1).inverse()*T10f(th1_1).inverse()*T60*T65f(th6_1).inverse()*T54f(th5_1).inverse();
    Vector3d Xhat43(T43m(0,0),T43m(1,0),T43m(2,0));
    double th4_1 = real(atan2(Xhat43(1), Xhat43(0)));
    
    T43m = T32f(th3_2).inverse()*T21f(th2_2).inverse()*T10f(th1_1).inverse()*T60*T65f(th6_2).inverse()*T54f(th5_2).inverse();
    Xhat43 << T43m(0,0),T43m(1,0),T43m(2,0);
    double th4_2 = real(atan2(Xhat43(1), Xhat43(0)));
    
    T43m = T32f(th3_3).inverse()*T21f(th2_3).inverse()*T10f(th1_2).inverse()*T60*T65f(th6_3).inverse()*T54f(th5_3).inverse();
    Xhat43 << T43m(0,0),T43m(1,0),T43m(2,0);
    double th4_3 = real(atan2(Xhat43(1), Xhat43(0)));
 
    T43m = T32f(th3_4).inverse()*T21f(th2_4).inverse()*T10f(th1_2).inverse()*T60*T65f(th6_4).inverse()*T54f(th5_4).inverse();
    Xhat43 << T43m(0,0),T43m(1,0),T43m(2,0);
    double th4_4 = real(atan2(Xhat43(1), Xhat43(0)));

    T43m = T32f(th3_5).inverse()*T21f(th2_5).inverse()*T10f(th1_1).inverse()*T60*T65f(th6_1).inverse()*T54f(th5_1).inverse();
    Xhat43 << T43m(0,0),T43m(1,0),T43m(2,0);
    double th4_5 = real(atan2(Xhat43(1), Xhat43(0)));
 
    T43m = T32f(th3_6).inverse()*T21f(th2_6).inverse()*T10f(th1_1).inverse()*T60*T65f(th6_2).inverse()*T54f(th5_2).inverse();
    Xhat43 << T43m(0,0),T43m(1,0),T43m(2,0);
    double th4_6 = real(atan2(Xhat43(1), Xhat43(0)));
 
    T43m = T32f(th3_7).inverse()*T21f(th2_7).inverse()*T10f(th1_2).inverse()*T60*T65f(th6_3).inverse()*T54f(th5_3).inverse();
    Xhat43 << T43m(0,0),T43m(1,0),T43m(2,0);
    double th4_7 = real(atan2(Xhat43(1), Xhat43(0)));
 
    T43m = T32f(th3_8).inverse()*T21f(th2_8).inverse()*T10f(th1_2).inverse()*T60*T65f(th6_4).inverse()*T54f(th5_4).inverse();
    Xhat43 << T43m(0,0),T43m(1,0),T43m(2,0);
    double th4_8 = real(atan2(Xhat43(1), Xhat43(0))) ;
    
    //Matrix of angle
    MatrixXd th(8,6);
    th <<   th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
            th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
            th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
            th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
            th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
            th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
            th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
            th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;
    //cout << th << endl;
    return th;
}
//------------------------------------------------------------------------------------------------------------------------------
//FINE INVERSE KINEMATICS
//------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------------------
//P2P MOTION PLAN
//------------------------------------------------------------------------------------------------------------------------------
MatrixXd p2pMotionPlan(pos_rot es, pos_rot ef,double minT, double maxT,double dt){
    VectorXd qEs = firstSolutionPossible(inverse(es));
    VectorXd qEf = firstSolutionPossible(inverse(ef));
    for(int i=0; i< qEs.size(); i++){
        if(qEs(i)>PI){
            qEs(i) = (-2)*PI + qEs(i);
        }
        if(qEf(i)>PI){
            qEf(i) = (-2)*PI + qEf(i);
        }
    }
    MatrixXd A(6,4);
    Matrix4d M;
    Vector4d b;
    Vector4d a;
    for(int i=0; i<qEs.size(); i++){
        M <<    1, minT, pow(minT, 2), pow(minT, 3),
                0, 1, 2*minT, 3*pow(minT, 2),
                1, maxT, pow(maxT, 2), pow(maxT, 3),
                0, 1, 2*maxT, 3*pow(maxT, 2);
        b << qEs(i), 0, qEf(i), 0;
        a = M.inverse()*b;
        A(i,0) = a(0);  A(i,1) = a(1);  A(i,2) = a(2);  A(i,3) = a(3);
    }

    MatrixXd Th(int(ceil((maxT-minT)/dt))+1,(qEs.size()+1));
    int index = 0;
    VectorXd temp_vet(7);
    double q;

    for(double t= minT; t<= (maxT+0.00001); t = t + dt){
        Th(index,0) = t;
        temp_vet[0] = t;
        for(int j=0; j< qEs.size(); j++){
            q = A(j,0) + (A(j,1)*t) + (A(j,2)*t*t) + (A(j,3)*t*t*t);
            Th(index,j + 1) = q;
        }
        index ++;
          
    }
    //cout << Th << endl;
    return Th;
}
//------------------------------------------------------------------------------------------------------------------------------
//FINE P2P MOTION PLAN
//------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------------------
//FUNZIONI COLLEGATE AL SUBSCRIBER
//------------------------------------------------------------------------------------------------------------------------------
/*
void stampa_posizione(const control_msgs::JointControllerState::ConstPtr& msg){
    ROS_INFO("Pos: %f",msg->process_value);
}
*/
void elbow_position(const control_msgs::JointControllerState::ConstPtr& msg){
    position[ELBOW] = msg->process_value;
    //cout << "Elbow pos: " << position[ELBOW] << endl;
}

void base_position(const control_msgs::JointControllerState::ConstPtr& msg){
    position[BASE] = msg->process_value;
    //cout << "Base pos: " << position[BASE] << endl;
}

void shoulder_position(const control_msgs::JointControllerState::ConstPtr& msg){
    position[SHOULDER] = msg->process_value;
    //cout << "Shoulder pos: " << position[SHOULDER] << endl;
}

void wrist1_position(const control_msgs::JointControllerState::ConstPtr& msg){
    position[WRIST1] = msg->process_value;
    //cout << "Wrist1 pos: " << position[WRIST1] << endl;
}

void wrist2_position(const control_msgs::JointControllerState::ConstPtr& msg){
    position[WRIST2] = msg->process_value;
    //cout << "Wrist2 pos: " << position[WRIST2] << endl;
}

void wrist3_position(const control_msgs::JointControllerState::ConstPtr& msg){
    position[WRIST3] = msg->process_value;
    //cout << "Wrist3 pos: " << position[WRIST3] << endl;
}
//------------------------------------------------------------------------------------------------------------------------------
//FINE FUNZIONI COLLEGATE AL SUBSCRIBER
//------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------------------
//CREA PUNTO 3D CON MATRICE DI ROTAZIONE
//------------------------------------------------------------------------------------------------------------------------------
pos_rot createPoint(double x, double y, double z, Matrix3d rot){
    Vector3d pos;
    pos << x, y, z;
    pos_rot temp(pos,rot);
    return temp;
}
//------------------------------------------------------------------------------------------------------------------------------
//FINE CREA PUNTO 3D CON MATRICE DI ROTAZIONE
//------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------------------
//CONTROLLA PRECISIONE POS E ROT
//------------------------------------------------------------------------------------------------------------------------------
bool controlPosition(pos_rot desideredPosition, double precision){
    pos_rot actualPosition = direct(position);
    Vector3d tempPos = desideredPosition.pos - actualPosition.pos;
    Matrix3d tempRot = desideredPosition.rot - actualPosition.rot;
    tempPos = tempPos.cwiseAbs();
    tempRot = tempRot.cwiseAbs();
    double maxPos = tempPos.maxCoeff();
    double maxRot = tempRot.maxCoeff();
    cout << "Position Error=" << maxPos << " " << "Rotation Error=" << maxRot << endl;
    if(maxPos > precision || maxRot > precision){
        return false;
    }else{ return true; }
}
//------------------------------------------------------------------------------------------------------------------------------
//FINE CONTROLLA PRECISIONE POS E ROT
//------------------------------------------------------------------------------------------------------------------------------

Matrix3d convertAngle2RotMatrix(Vector3d angle){
    double roll = angle(2); double yaw = angle(1); double pitch = angle(0);
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    
    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    Matrix3d rotationMatrix = q.matrix();
    return rotationMatrix;
}


double angle(pos_rot p1, pos_rot p2){
    Vector2d v1, v2, origin;
    double a;
    v1 << -p1.pos(0), -p1.pos(1);
    v2 << -p2.pos(0), -p2.pos(1);
    origin << 1, 0;
    if ((v1(1) > 0 && v2(1) > 0) || (v1(1) < 0 && v2(1) < 0)){
        a = acos(v1.dot(v2)/(v1.norm()*v2.norm()));
    }else{
        double a1 = acos(v1.dot(origin)/(v1.norm()));
        double a2 = acos(v2.dot(origin)/(v2.norm()));
        a = a1 + a2;
    }
    return a;
}

pos_rot pickPosition(Vector3d pos, double angle, int mode){
    double pitch, yaw, roll;
    yaw=-PI;
    double x = pos(0); double y = pos(1); double z = pos(2);

    if(mode==0){
        pitch = 0; yaw=-PI; roll=angle;
        return createPoint(x,y,z,convertAngle2RotMatrix(Vector3d(pitch, yaw, roll)));
    }else if(mode==1){
        pitch = -PI/4;
        roll = angle;
    }else if(mode==2){
        pitch = PI/4;
        roll = PI;
    }else if(mode==3){
        pitch = -PI/4;
        roll = PI/2;
    }else if(mode==4){
        pitch = PI/4;
        roll = PI/2;
    }
   
    cout << pitch << " " << yaw << " " << roll << endl;
    return createPoint(x,y,z,convertAngle2RotMatrix(Vector3d(pitch, yaw, roll)));
}

int main(int argc, char **argv)
{

    cout << "node running..." << endl;
    //------------------------------------------------------------------------------------------------------------------------------
    //INIZIALIZZAZIONE
    //------------------------------------------------------------------------------------------------------------------------------
    ros::init(argc, argv, "simple_node");

    ros::NodeHandle node;

    //Nodi per comandi posizione
    ros::NodeHandle elbow;
    ros::NodeHandle base;
    ros::NodeHandle shoulder;
    ros::NodeHandle wrist1;
    ros::NodeHandle wrist2;
    ros::NodeHandle wrist3;
    //Nodi per dati posizione
    ros::NodeHandle elbow_data;
    ros::NodeHandle base_data;
    ros::NodeHandle shoulder_data;
    ros::NodeHandle wrist1_data;
    ros::NodeHandle wrist2_data;
    ros::NodeHandle wrist3_data;

    //Publisher
    ros::Publisher command_elbow = elbow.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
    ros::Publisher command_base = base.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command",1000); 
    ros::Publisher command_shoulder = shoulder.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command",1000);
    ros::Publisher command_wrist1 = wrist1.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command",1000);
    ros::Publisher command_wrist2 = wrist2.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command",1000);
    ros::Publisher command_wrist3 = wrist3.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command",1000);
    ros::Publisher pos_reached = node.advertise<std_msgs::String>("manager/in_place", 1); 
    //Subscriber
    ros::Subscriber pos_elbow = elbow_data.subscribe("/elbow_joint_position_controller/state", 1000, elbow_position);
    ros::Subscriber pos_base = base_data.subscribe("/shoulder_pan_joint_position_controller/state",1000, base_position); 
    ros::Subscriber pos_shoulder = shoulder_data.subscribe("/shoulder_lift_joint_position_controller/state",1000, shoulder_position);
    ros::Subscriber pos_wrist1 = wrist1_data.subscribe("/wrist_1_joint_position_controller/state",1000, wrist1_position);
    ros::Subscriber pos_wrist2 = wrist2_data.subscribe("/wrist_2_joint_position_controller/state",1000, wrist2_position);
    ros::Subscriber pos_wrist3 = wrist3_data.subscribe("/wrist_3_joint_position_controller/state",1000, wrist3_position);
    //------------------------------------------------------------------------------------------------------------------------------
    //FINE INIZIALIZZAZIONE
    //------------------------------------------------------------------------------------------------------------------------------
    

    //------------------------------------------------------------------------------------------------------------------------------
    //MUOVI JOINTS
    //------------------------------------------------------------------------------------------------------------------------------
    auto moveJoints = [command_base,command_elbow,command_shoulder,command_wrist1,command_wrist2,command_wrist3] (VectorXd v){
        std_msgs::Float64 msg[N_JOINT];
        msg[BASE].data = v(BASE);
        command_base.publish(msg[BASE]);
        msg[SHOULDER].data = v(SHOULDER);
        command_shoulder.publish(msg[SHOULDER]);
        msg[ELBOW].data = v(ELBOW);
        command_elbow.publish(msg[ELBOW]);
        msg[WRIST1].data = v(WRIST1);
        command_wrist1.publish(msg[WRIST1]);
        msg[WRIST2].data = v(WRIST2);
        command_wrist2.publish(msg[WRIST2]);
        msg[WRIST3].data = v(WRIST3);
        command_wrist3.publish(msg[WRIST3]);
    };
    //------------------------------------------------------------------------------------------------------------------------------
    //FINE MUOVI JOINTS
    //------------------------------------------------------------------------------------------------------------------------------


    //------------------------------------------------------------------------------------------------------------------------------
    //POINT 2 POINT MOTION PLAN COMPLETE
    //------------------------------------------------------------------------------------------------------------------------------
    auto p2pMotionPlanComplete = [moveJoints](pos_rot partenza, pos_rot arrivo,double min, double max, double dt, int loopRate, double precision){
        MatrixXd position = p2pMotionPlan(partenza,arrivo,min,max,dt);
        //cout << "Dimensione Matrice " << position.rows() << endl;
        cout << "-----------------------------------------" << endl;
        //cout << "Partenza: " << endl;
        //partenza.stampa();
        //cout << "Arrivo: "  << endl;
        //arrivo.stampa();
        VectorXd vetTemp(6);
        ros::Rate loop_rate(loopRate);

        int index = 0;
        while (ros::ok())
        {   
            vetTemp << position(index,1), position(index,2), position(index,3), 
                        position(index,4), position(index,5), position(index,6);
            moveJoints(vetTemp);
            ros::spinOnce();
            loop_rate.sleep();
            index ++;
            if(index > int(ceil((max-min)/dt))){
                index = 0;
                controlPosition(arrivo,0.01);
                while(ros::ok()){
                    moveJoints(vetTemp);
                    ros::spinOnce();
                    loop_rate.sleep();
                    index++;
                    if(index>150){
                        break;
                    } 
                }
                break;
            }
        }
        return controlPosition(arrivo, precision);
    };
    //------------------------------------------------------------------------------------------------------------------------------
    //FINE POINT 2 POINT MOTION PLAN COMPLETE
    //------------------------------------------------------------------------------------------------------------------------------
    
    
    //------------------------------------------------------------------------------------------------------------------------------
    //SEZIONE INIZIALIZZAZIONE ROBOT
    //------------------------------------------------------------------------------------------------------------------------------
    auto initialize = [p2pMotionPlanComplete](){
        ros::Rate loop_rate(100);
        int index = 0;
        while (ros::ok())
        {   
            ros::spinOnce();
            loop_rate.sleep();
            index ++;
            if(index >= INITIALIZE_CYCLE){
                break;
            }
        }
        p2pMotionPlanComplete(direct(position),pickPosition(Vector3d(0, -0.5, 0.3), 0, 0),0,1,0.001,1000,0.01);
    };
       
    initialize();

    //ros::Subscriber gripper_state = node.subscribe<std_msgs::String>("gripper_state", 1, );

    string current_t;
    ros::Subscriber sub = node.subscribe<std_msgs::String>("manager_positions", 10, [p2pMotionPlanComplete, pos_reached, current_t, node](const std_msgs::String::ConstPtr& msg) mutable{
        double x,y,z, rot;
        string t;
        int upward = 1, side = 0;

        string data = msg->data.c_str();
        vector<string> inputs;
        stringstream ss(data);
        string it;
        char delim = ' ';

        cout << data << endl;
        while (getline (ss, it, delim)) {
            inputs.push_back (it);
        }
        
        x = std::stod(inputs[0]);
        y = std::stod(inputs[1]);
        z = std::stod(inputs[2]);
        rot = std::stod(inputs[3]);
        t = inputs[4];
        if(inputs.size() == 7) {
            upward = stoi(inputs[5]);
            side = stoi(inputs[6]);     // -1 -> on the left, 1 -> on the right
        }

        std_msgs::String val;

        // t -1 when receiving camera data 
        if (t == "-1") {
            Vector3d actual = direct(position).pos;
            double xx = actual(0)-x, yy = actual(1)+y;
            Vector3d n(xx,yy, 0.01);
            //cout << "NUOVA: " << n << endl;
            if (upward != 1) {
                double offset = 0.0;
                if(current_t == "X1-Y1-Z2")
                    if(side == 1) //right
                        offset = (rot>0) ? M_PI : 0.0;
                    else
                        offset = (rot<0) ? PI : 0.0;
                if(current_t == "X1-Y2-Z1" || current_t == "X1-Y2-Z2" || current_t == "X1-Y2-Z2-C" || current_t == "X1-Y2-Z2-T" || current_t == "X2-Y2-Z2" || current_t == "X2-Y2-Z2-F")
                    offset = (side==1) ? -M_PI_2 : M_PI_2;
                rot += offset;
            }
           
            // trova info mattoncino
            map<string, Brick>::iterator itr = stands.find(current_t);
            if (upward == 0)
            {           
                // PEZZI LUNGHI (y3 e y4) 
                if((current_t == "X1-Y3-Z2" || current_t ==  "X1-Y3-Z2-F" || current_t == "X1-Y4-Z1" || current_t == "X1-Y4-Z2")) {
                    
                    pos_rot piece = pickPosition(Vector3d(xx,yy,0.01), rot, 1);
                    pos_rot overPieceRotated = pickPosition(Vector3d(xx,yy,0.3), 0, 2);
                    pos_rot pieceRotated = pickPosition(Vector3d(xx,yy,itr->second.length/2 + 0.01), 0, 2);
                    //pos_rot pieceRotated2 = pickPosition(Vector3d(xx,yy,itr->second.length/2), 0, 3);
                    pos_rot pieceRotated2 = pickPosition(Vector3d(xx,yy,0.15), 0, 2);
                    pos_rot pieceRotated3 = pickPosition(Vector3d(xx,yy,itr->second.height + 0.01), 0, 2);
                    pos_rot overPiece = pickPosition(Vector3d(xx,yy,0.3), 0, 0);
                    
                    pos_rot pieceCorrect = pickPosition(Vector3d(xx,yy,itr->second.length-0.02), side*M_PI_2, 1);
                    pos_rot overPieceCorrect = changeZ(pieceCorrect, 0.2);

                    /////////////// MOVIMENTO 1 //////////////////////
                    //prendere il pezzo a 45 dall'esterno
                    p2pMotionPlanComplete(direct(position),piece,0,1,0.001,1000,0.01);

                    //chiusura pinza
                    val.data = "2";
                    pos_reached.publish(val);
                    std_msgs::BoolConstPtr shared;
                    std_msgs::Bool msgB;
                    shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                    if (shared != NULL) {
                        cout << *shared << endl;
                    }

                    //ruotare il pezzo correttamente
                    p2pMotionPlanComplete(direct(position),overPieceRotated,0,1.5,0.001,1000,0.01);
                    //posizionare il pezzo ruotato correttamente
                    p2pMotionPlanComplete(direct(position),pieceRotated,0,1.5,0.001,1000,0.01);

                    //Apertura pinza
                    val.data = "3";
                    pos_reached.publish(val);
                    shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                    if (shared != NULL) {
                        cout << *shared << endl;
                    }
                    //alzare poco la pinza
                    p2pMotionPlanComplete(direct(position),pieceRotated2,0,0.5,0.001,1000,0.01);


                    /////////////// MOVIMENTO 2 //////////////////////
                    p2pMotionPlanComplete(direct(position),overPieceCorrect,0,1,0.001,1000,0.01);
                    //prendi da sopra a -45 (asse y) +-PI/2 asse x
                    p2pMotionPlanComplete(direct(position),pieceCorrect,0,1,0.001,1000,0.01);
                    //chiusura pinza
                    val.data = "2";
                    pos_reached.publish(val);
                    shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                    if (shared != NULL) {
                        cout << *shared << endl;
                    }

                    //ruotare il pezzo correttamente
                    p2pMotionPlanComplete(direct(position),overPieceRotated,0,1.5,0.001,1000,0.01);
                    //posizionare il pezzo ruotato correttamente
                    p2pMotionPlanComplete(direct(position),pieceRotated3,0,1.5,0.001,1000,0.01);

                    //Apertura pinza
                    val.data = "3";
                    pos_reached.publish(val);
                    shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                    if (shared != NULL) {
                        cout << *shared << endl;
                    }

                    //alzare poco la pinza
                    p2pMotionPlanComplete(direct(position),pieceRotated2,0,0.5,0.001,1000,0.01);
                    //posizionarsi sopra il pezzo
                    p2pMotionPlanComplete(direct(position),overPiece,0,1,0.001,1000,0.01);

                    //attendi assestamento
                    ros::Duration(3).sleep();
                    val.data = "1";
                    pos_reached.publish(val); 
                }
                else {      // PEZZI CORTI (y1 e y2)
                    pos_rot piece = pickPosition(Vector3d(xx,yy,0.01), rot, 1);
                    pos_rot overPieceRotated = pickPosition(Vector3d(xx,yy,0.3), 0, 2);
                    pos_rot pieceRotated = pickPosition(Vector3d(xx,yy,0.02), 0, 2);
                    pos_rot pieceRotated2 = pickPosition(Vector3d(xx,yy,0.15), 0, 2);
                    pos_rot overPiece = pickPosition(Vector3d(xx,yy,0.3), 0, 0);

                    //prendere il pezzo a 45 dall'esterno
                    p2pMotionPlanComplete(direct(position),piece,0,1,0.001,1000,0.01);

                    //chiusura pinza
                    val.data = "2";
                    pos_reached.publish(val);
                    std_msgs::BoolConstPtr shared;
                    std_msgs::Bool msgB;
                    shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                    if (shared != NULL) {
                        cout << *shared << endl;
                    }

                    //ruotare il pezzo correttamente
                    p2pMotionPlanComplete(direct(position),overPieceRotated,0,1.5,0.001,1000,0.01);
                    //posizionare il pezzo ruotato correttamente
                    p2pMotionPlanComplete(direct(position),pieceRotated,0,1.5,0.001,1000,0.01);

                    //Apertura pinza
                    val.data = "3";
                    pos_reached.publish(val);
                    shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                    if (shared != NULL) {
                        cout << *shared << endl;
                    }
                    //alzare poco la pinza
                    p2pMotionPlanComplete(direct(position),pieceRotated2,0,0.5,0.001,1000,0.01);

                    //posizionarsi sopra il pezzo
                    p2pMotionPlanComplete(direct(position),overPiece,0,1,0.001,1000,0.01);

                    //attendi assestamento
                    ros::Duration(3).sleep();
                    val.data = "1";
                    pos_reached.publish(val);          
                }    
            } else if (upward == -1) {  // rivolti in basso
                pos_rot piece = pickPosition(Vector3d(xx,yy,0.03), rot, 1);
                pos_rot overPieceRotated = pickPosition(Vector3d(xx,yy,0.3), 0, 4);
                pos_rot pieceRotated = pickPosition(Vector3d(xx,yy,itr->second.width/2 + 0.01), 0, 4);
                if(current_t == "X1-Y3-Z2" || current_t ==  "X1-Y3-Z2-F" || current_t == "X1-Y4-Z1" || current_t == "X1-Y4-Z2")
                    pieceRotated = pickPosition(Vector3d(xx,yy,itr->second.length/2 + 0.01), 0, 4);
                pos_rot pieceRotated2 = pickPosition(Vector3d(xx,yy,0.25), 0, 4);
                pos_rot pieceRotated21 = pickPosition(Vector3d(xx,yy,0.25), 0, 3);
                pos_rot pieceRotated3 = pickPosition(Vector3d(xx,yy,itr->second.width/2 + 0.01), 0, 3);
                if(current_t == "X1-Y3-Z2" || current_t ==  "X1-Y3-Z2-F" || current_t == "X1-Y4-Z1" || current_t == "X1-Y4-Z2")
                    pieceRotated3 = pickPosition(Vector3d(xx,yy,itr->second.length/2 + 0.01), 0, 3);
                pos_rot pieceRotated31 = pickPosition(Vector3d(xx,yy,0.25), 0, 4);
                pos_rot pieceRotated4 = pickPosition(Vector3d(xx,yy,0.03), 0, 4);
                
                pos_rot overPiece = pickPosition(Vector3d(xx,yy,0.3), 0, 0);

                //prendere il pezzo a 45 dall'esterno
                p2pMotionPlanComplete(direct(position),piece,0,2,0.001,1000,0.01);
                cout << direct(position).pos << endl;
                //chiusura pinza
                val.data = "2";
                pos_reached.publish(val);
                std_msgs::BoolConstPtr shared;
                std_msgs::Bool msgB;
                shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                if (shared != NULL) {
                    cout << *shared << endl;
                }

                //ruotare il pezzo correttamente
                p2pMotionPlanComplete(direct(position),overPieceRotated,0,1.5,0.001,1000,0.01);
                //posizionare il pezzo ruotato correttamente
                p2pMotionPlanComplete(direct(position),pieceRotated,0,1.5,0.001,1000,0.01);

                //Apertura pinza
                val.data = "3";
                pos_reached.publish(val);
                shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                if (shared != NULL) {
                    cout << *shared << endl;
                }
                //alzare poco la pinza
                p2pMotionPlanComplete(direct(position),pieceRotated2,0,0.5,0.001,1000,0.01);	
                p2pMotionPlanComplete(direct(position),pieceRotated21,0,1,0.001,1000,0.01);
                //p2pMotionPlanComplete(direct(position),pieceRotated41,0,1,0.001,1000,0.01);
                //ruotare pinza
                p2pMotionPlanComplete(direct(position),pieceRotated3,0,1,0.001,1000,0.01);
                
                //chiusura pinza
                val.data = "2";
                pos_reached.publish(val);
                shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                if (shared != NULL) {
                    cout << *shared << endl;
                }

                //ruotare pezzo
                p2pMotionPlanComplete(direct(position),pieceRotated31,0,1,0.001,1000,0.01);
                p2pMotionPlanComplete(direct(position),pieceRotated4,0,1,0.001,1000,0.01);

                //Apertura pinza
                val.data = "3";
                pos_reached.publish(val);
                shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                if (shared != NULL) {
                    cout << *shared << endl;
                }
                
                //posizionarsi sopra il pezzo
                p2pMotionPlanComplete(direct(position),overPiece,0,1,0.001,1000,0.01);

                //attendi assestamento
                ros::Duration(3).sleep();
                val.data = "1";
                pos_reached.publish(val);          
            } else { //MOVIMENTI FINALI IN COMUNE (verso lo stand)
                // abbassati
                p2pMotionPlanComplete(direct(position),pickPosition(n, rot, 0),0,2.0,0.0005,1000,0.01);
                // chiedi a manager chiudi pinza
                val.data = "2";
                pos_reached.publish(val);
                std_msgs::BoolConstPtr shared;
                std_msgs::Bool msgB;
                shared = ros::topic::waitForMessage<std_msgs::Bool>("gripper_state", node);
                if (shared != NULL) {
                    cout << *shared << endl;
                }
                            
                // alzati
                p2pMotionPlanComplete(direct(position),changeZ(direct(position), 0.3),0,1,0.001,1000,0.01);

                if (ASSIGNMENT4 == 1) {
                    // calcola posizione finale
                    pos_rot final = pickPosition(Vector3d(get<1>(ass4[ind]), get<2>(ass4[ind]), get<3>(ass4[ind]) +0.01), 0, 0);
                    // muovi sopra
                    double tempo = ceil((angle(direct(position), final) / PI ) *30)/10;
                    p2pMotionPlanComplete(direct(position),changeZ(final, 0.4),0,tempo,0.001,1000,0.01);
                    // abbassati
                    p2pMotionPlanComplete(direct(position),final,0,3.0,0.001,1000,0.01);
                    // chiedi attach & apri pinza
                    val.data = "4";
                    pos_reached.publish(val);
                    ++ind;
                    // attendi apertura pinza
                    ros::Duration(3).sleep();
                    p2pMotionPlanComplete(direct(position),changeZ(final, 0.3),0,1,0.001,1000,0.01);
                    
                } else {
                    if (itr != stands.end()) {
                        Brick b = itr->second;
                        // calcola posizione finale stand
                        pos_rot stand = pickPosition(Vector3d(b.x, 0.5, (b.count * b.height) +0.01), 0, 0);
                        itr->second.count = ++b.count;
                        // muovi stand - sopra
                        double tempo = ceil((angle(direct(position), stand) / PI ) *30)/10;
                        p2pMotionPlanComplete(direct(position),changeZ(stand, 0.4),0,tempo,0.001,1000,0.01);
                        // abbassati su stand
                        p2pMotionPlanComplete(direct(position),stand,0,2.0,0.001,1000,0.01);
                        // chiedi attach & apri pinza
                        val.data = "4";
                        pos_reached.publish(val);
                        // attendi apertura pinza
                        ros::Duration(3).sleep();
                        p2pMotionPlanComplete(direct(position),changeZ(stand, 0.3),0,1,0.001,1000,0.01);
                    } else {
                        cout << "type <" << t << "> not found" << endl;
                    }  
                }
            }

        }
        else {
            current_t = t;

            pos_rot p1 = pickPosition(Vector3d(-x,-y, 0.3), 0, 0);
            double tempo = ceil((angle(direct(position), p1) / PI ) *30)/10;
            p2pMotionPlanComplete(direct(position),p1,0,tempo,0.001,1000,0.01);
            p2pMotionPlanComplete(direct(position),p1,0,0.5,0.001,1000,0.01);
            //attendi assestamento
            ros::Duration(2).sleep();
            val.data = "1";
            pos_reached.publish(val);                      
        }
        
    });


    
    ros::spin();

    return 0;
}