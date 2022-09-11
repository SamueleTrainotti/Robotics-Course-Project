#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <list>
#include <cmath>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"

#define DEGREEPERRADIANS 57.2957


using namespace std;

//visuale perpendicolare al terreno
/*#define XGAZEBO 1 // x quadrato lato in alto a sinistra
#define YGAZEBO 1 // y quadrato lato in alto a sinistra
#define ALTEZZA 100
#define LARGHEZZA 100*/

//visuale inclinata rispetto al terreno
#define X2GAZEBO 0.33 // x trapezio lato in basso a sinistra
#define Y2GAZEBO 0.9 // y trapezio lato in basso a sinistra
#define ALTEZZACAMERA 0.5
#define CAMERAINIZIOIMMAGINE 0.12
#define CAMERAFINEIMMAGINE 2
#define LARGHEZZASTRETTATRAP 0.66
#define LARGHEZZALARGATRAP 2.8

typedef struct {
    string id;
    float x;
    float y;
    float w;
    float h;
    float xGazebo;
    float yGazebo;
} Oggetto;

string convertiId (int num){
    string risultato;
    switch (num)
    {
    case 0:
        risultato = "X1-Y1-Z2";
        break;
    case 1:
        risultato = "X1-Y2-Z1";
        break;
    case 2:
        risultato = "X1-Y2-Z2";
        break;
    case 3:
        risultato = "X1-Y2-Z2-C";
        break;
    case 4:
        risultato = "X1-Y2-Z2-T";
        break;
    case 5:
        risultato = "X1-Y3-Z2";
        break;
    case 6:
        risultato = "X1-Y3-Z2-F";
        break;
    case 7:
        risultato = "X1-Y4-Z1";
        break;
    case 8:
        risultato = "X1-Y4-Z2";
        break;
    case 9:
        risultato = "X2-Y2-Z2";
        break;
    case 10:
        risultato = "X2-Y2-Z2-F";
        break;
    default:
        printf("errore id classe non trovato \n");
        break;
    }
    return risultato;
}

int main(int argc, char **argv){
    system("sh ~/catkin_ws/run2.sh");
    system("sh ~/catkin_ws/run.sh");

    ros::init(argc, argv, "detector");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("detector", 10, true);
    ros::Rate loop_rate(1);

    string linea;
    const char separatore = ' ';
    string lineaArr[5];
    string val;
    Oggetto newOgg;
    list<Oggetto> listaOggetti;
    int i;
    ifstream myDati;
    //myDati.open("yolov5/prova.txt");
    myDati.open("src/yolov5/runs/detect/exp/labels/left0000.txt");

    //lettura dati da file e memorizzazione in listaOggetti
    if(myDati.is_open()){
        while (getline(myDati, linea)){
            cout << linea << '\n';
            stringstream ss(linea);
            i = 0;
            while(getline(ss, val, separatore)){
                lineaArr[i] = val;
                i++;
            }
            newOgg.id = convertiId(stoi(lineaArr[0]));
            newOgg.x = stof(lineaArr[1]);
            newOgg.y = stof(lineaArr[2]);
            newOgg.w = stof(lineaArr[3]);
            newOgg.h = stof(lineaArr[4]);
            cout << newOgg.id << newOgg.x << '\n';
            cout << newOgg.y << newOgg.w << newOgg.h << '\n';
            listaOggetti.push_back(newOgg);
        }
        myDati.close();
    } else{
        cout << "errore" << '\n';
    }

    //visuale perpendicolare al terreno
    /*for(auto it = listaOggetti.begin(); it != listaOggetti.end(); ++it){
        (*it).xGazebo = XGAZEBO + (LARGHEZZA * (*it).w);
        (*it).yGazebo = YGAZEBO + (ALTEZZA * (*it).h);
        cout << "coordinate = " << (*it).xGazebo << ' ' << (*it).yGazebo <<'\n';
    }*/

    //visuale inclinata rispetto al terreno
    float AD = sqrt((pow(CAMERAINIZIOIMMAGINE, 2) + pow(ALTEZZACAMERA, 2)));
    float aa = asin((CAMERAINIZIOIMMAGINE / AD)) * DEGREEPERRADIANS;
    float DF = sqrt((pow(CAMERAFINEIMMAGINE, 2) + pow(ALTEZZACAMERA, 2)));
    float abab = asin((CAMERAFINEIMMAGINE / DF)) * DEGREEPERRADIANS;
    float bb = abab - aa;
    float tt = (180 - bb) / 2;
    float a2 = aa + 90 - tt;
    float AE = sqrt((pow(AD, 2) + pow(AD, 2) - (2 * AD * AD * cos((bb / DEGREEPERRADIANS)))));
    float AB;
    float BD;
    float b1;
    float b2;
    float c1;
    float AC;
    float DIFF = LARGHEZZALARGATRAP - LARGHEZZASTRETTATRAP;
    float AF = CAMERAFINEIMMAGINE - CAMERAINIZIOIMMAGINE;
    float NO;
    float SOTT;
    float POSITIONX;

    std_msgs::String msg;
    std::stringstream ss;
    ss.precision(6);
    for(auto it = listaOggetti.begin(); it != listaOggetti.end(); ++it){
        AB = AE * (1 - (*it).y);
        BD = sqrt((pow(AB, 2) + pow(AD, 2) - (2 * AB * AD * cos((tt / DEGREEPERRADIANS)))));
        b1 = acos(((pow(AB, 2) + pow(BD, 2) - pow(AD, 2)) / (2 * AB * BD))) * DEGREEPERRADIANS;
        b2 = 180 - b1;
        c1 = 180 - (a2 + b2);
        AC = (AB / sin(c1 / DEGREEPERRADIANS)) * sin(b2 / DEGREEPERRADIANS); //Y
        NO = LARGHEZZASTRETTATRAP + (DIFF * (AC / AF));
        SOTT = (NO - LARGHEZZASTRETTATRAP) / 2;
        POSITIONX = NO * (*it).x; //X
        (*it).xGazebo = X2GAZEBO + SOTT - POSITIONX;
        (*it).yGazebo = Y2GAZEBO - AC;

        //ss <<" coordinate = " << (*it).xGazebo <<" "<< (*it).yGazebo <<'\n';
        ss << (*it).id << " " << static_cast<double>((*it).xGazebo) << " " << static_cast<double>((*it).yGazebo) << ',';
    }

    if (ss.str().empty()) {
        cout << "vuoto B";
    } else {
        cout << ss.str();
        msg.data = ss.str();
        pub.publish(msg);
    }

    ros::Duration(2).sleep();
    return 0;
}
