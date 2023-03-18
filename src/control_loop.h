#include <iostream>
#include <fstream>
#include "boost/thread.hpp"

using namespace std;

class CONTROLLER {
    public:
        CONTROLLER(float kp, float ki, float kd);
        
        void loop();                //Main loop function
        void system_start();       //start the system
        void set_xdes(double x);   //member to set the desired value

    private:
        double _xdes; //desired value to reach
        double _xmes; //current value of my system
        float _eps; //error threshold
        
        double _kp; // PID gains 
        double _kd; // PID gains
        double _ki; // PID gains

        bool _first_des_value;


};