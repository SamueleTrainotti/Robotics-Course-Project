#include <ros/ros.h>
#include <ros/package.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "control_msgs/JointControllerState.h"
#include <iostream>
#include <math.h>
#include <complex.h>
#include <signal.h>
#include <chrono>
#include <thread>
#include <algorithm>

#include <gazebo_ros_link_attacher/Attach.h>

#define ASSIGNMENT4 1

using namespace std;


ros::Publisher position_command, pos_reached, gripper_cmd;
double rot = 0;
int upward = 1;
int side = 0;
ros::ServiceClient attacher;

string current;
map<string, int> counter = {
    {"X1-Y1-Z2", 0},
    {"X1-Y2-Z1", 0},
    {"X1-Y2-Z2", 0},
    {"X1-Y2-Z2-C", 0},
    {"X1-Y2-Z2-T", 0},
    {"X1-Y3-Z2", 0},
    {"X1-Y3-Z2-F", 0},
    {"X1-Y4-Z1", 0},
    {"X1-Y4-Z2", 0},
    {"X2-Y2-Z2", 0},
    {"X2-Y2-Z2-F", 0},
};
int ind = 0;
vector<string> ass4 = { "X2-Y2-Z2_0", "X2-Y2-Z2_1", "X2-Y2-Z2-F_0" };

struct Lego {
    string type;
    double x, y;
    Lego(string t, double X, double Y):type(t),x(X),y(Y) {};
    Lego() {}
};
vector<Lego> legoVec;
vector<Lego>::const_iterator lego_itr;


void sendNext() {   // inizia processamento lego
    if (lego_itr != legoVec.cend()) {
        current = lego_itr->type;

        std_msgs::String pos_msg;
        std::stringstream send;        
        send << lego_itr->x << " ";
        send << lego_itr->y << " ";
        send << 0.3 << " ";
        send << rot << " ";
        send << current;
        pos_msg.data = send.str();
        position_command.publish(pos_msg);

    lego_itr++;
    } else
        ROS_INFO("LEGO FINITI!");
}
void getDetections(const std_msgs::String::ConstPtr& msg) {
    string data = msg->data.c_str();
    vector<string> inputs;
    stringstream ss(data);
    string it, l;
    char delim = ',';
    char space = ' ';

    ss.precision(6);
    while (getline (ss, it, delim)) {
        stringstream item(it);
        while (getline (item, l, space)) {
            inputs.push_back (l);
        }           
    }
    
    // resize vec to fit lego count
    legoVec.resize(inputs.size()/3);

    for (int i=0; i < inputs.size(); ++i) {
        switch (i%3)
        {
            case 0:
                legoVec[i/3].type = inputs[i];
                break;
            case 1:
                legoVec[i/3].x = stod(inputs[i]);
                break;
            case 2:
                legoVec[i/3].y = stod(inputs[i]);
                break;
        }
    }
    for (auto &&s:legoVec)
    {
        cout << s.type << endl;
    }
    if(ASSIGNMENT4 == 1) {
        sort(legoVec.begin(), legoVec.end(), [](Lego s1, Lego s2){
            /*vector<string>::iterator it1, it2;
            it1 = find(ass4.begin(), ass4.end(), s1.type);
            it2 = find(ass4.begin(), ass4.end(), s2.type);*/
            return s1.type < s2.type;
        });
    }

    for (auto &&s:legoVec)
    {
        cout << s.type << endl;
    }
    
    lego_itr = legoVec.cbegin();
    sendNext();
}
void setRot(const std_msgs::String::ConstPtr& msg) {
    string data = msg->data.c_str();
    vector<string> inputs;
    stringstream ss(data);
    string it;
    char space = ' ';

    ss.precision(6);
    while (getline (ss, it, space)) {
        stringstream item(it);
        inputs.push_back(it);
    }
    
    rot = stod(inputs[0]);
    upward = stoi(inputs[1]);
    side = stoi(inputs[2]);
}
void sendErr(const std_msgs::String::ConstPtr& msg) {
    string data = msg->data.c_str();
    vector<string> inputs;
    stringstream ss(data);
    string it;
    char delim = ' ';

    while (getline (ss, it, delim)) {
        inputs.push_back (it);
    }

    double newX, newY;
    newX = stod(inputs[0]) * 0.0008;
    newY = (stod(inputs[1])-65) * 0.0008;
    
    std_msgs::String pos_msg;
    std::stringstream send;
    send << newX << " ";
    send << newY << " ";
    send << 0.3 << " ";
    send << rot << " ";
    send << "-1" << " ";
    send << upward << " ";
    send << side;
    pos_msg.data = send.str();
    position_command.publish(pos_msg);
}
void inPlace(const std_msgs::String::ConstPtr& msg) {
    string data = msg->data.c_str();
    if (data == "1") {
        // ottieni dati errore da camera
        std_msgs::String val;
        val.data = "1";
        pos_reached.publish(val);
    } 
    if (data == "2" ) {
        // chiudi gripper
        std_msgs::Bool b;
        b.data = false;
        gripper_cmd.publish(b);
    }
    if (data == "3" ) {
        // apri gripper
        std_msgs::Bool b;
        b.data = true;
        gripper_cmd.publish(b);
    }
    if (data == "4" ) {
        // attach
        map<string, int>::iterator it = counter.find(current);
        gazebo_ros_link_attacher::Attach srv;
        
        if(ASSIGNMENT4 == 1) {
            if (ind == 0)
                srv.request.model_name_1 = "ground_plane";
            else 
                srv.request.model_name_1 = ass4[ind-1];            
            srv.request.link_name_1 = "link";
            srv.request.model_name_2 = ass4[ind];
            srv.request.link_name_2 = "link";

            if (attacher.call(srv))
                ROS_INFO("attacher called");
            else
                ROS_ERROR("Failed to call service ");
            ++ind;
        } else if(it->second != 0) {
            srv.request.model_name_1 = current + "_" + to_string(it->second-1);
            srv.request.link_name_1 = "link";
            srv.request.model_name_2 = current + "_" + to_string(it->second);
            srv.request.link_name_2 = "link";

            if (attacher.call(srv))
                ROS_INFO("attacher called");
            else
                ROS_ERROR("Failed to call service ");
        }
        it->second++;

        // apri gripper
        std_msgs::Bool b;
        b.data = true;
        gripper_cmd.publish(b);
        // attendi apertura pinza e salita (3 + 1 secondi)
        
        ros::Duration(4).sleep();

        // invia prossimo lego
        sendNext();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manager_node");

    srand(time(NULL));

    //Nodi 
    ros::NodeHandle camera, gripper, kinematics, links, detector, node;

    //link service
    attacher = links.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

    // ottieni errore da camera
    pos_reached = camera.advertise<std_msgs::String>("stream_images_node/in_place", 1); 
    // comandi pinza 
    gripper_cmd = camera.advertise<std_msgs::Bool>("gripper_commands", 1); 

    // posizione lego rilvati
    ros::Subscriber detector_data = detector.subscribe<std_msgs::String>("detector", 1, getDetections);
    // informazioni rotazione (verso l'alto/basso/sdraiati e a destra/sinistra)
    ros::Subscriber camera_data = camera.subscribe<std_msgs::String>("stream_images_node/rotation", 10, setRot);
    // errore pezzi --> riposiziona per centrare il lego
    ros::Subscriber err_pos_data = camera.subscribe<std_msgs::String>("stream_images_node/err_pos", 1, sendErr);
    // posizione raggiunta --> indica quanto il braccio è fermo
    ros::Subscriber place_data = kinematics.subscribe<std_msgs::String>("manager/in_place", 1, inPlace);

    //Comandi posizione
    position_command = kinematics.advertise<std_msgs::String>("manager_positions", 10);


    ros::spin();
    return 0;
}