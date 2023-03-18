#include "control_loop.h"

//We can use the class constructor to set parameters
CONTROLLER::CONTROLLER(float kp, float kd, float ki) {
    _kp = kp;
    _ki = ki;
    _kd = kd;

    _eps = 0.001;

    _first_des_value = false;

    CONTROLLER::system_start();
    
    //Run thread of our system
    boost::thread loop_t( &CONTROLLER::loop, this);                //Main control loop
    
    //boost::thread NAME_THREAD( &CLASS_OF_THE_FUNCT::NAME_OF_THE_FUNCTION, CONTEXT, PARAMETERS )
}


//Sense: get input to change the state of our System
void CONTROLLER::set_xdes(double x) {
    _xdes = x;

    _first_des_value = true; //start controller
}


//Random initial value
void CONTROLLER::system_start() {
    
    //Generate a random initial value
    srand((unsigned int)time(NULL));
    float random = ((float) rand()) / (float) RAND_MAX;
    _xmes = random;

}

void CONTROLLER::loop() {

    ofstream myfile;

    double e = 0.0;
    double de = 0.0;
    double ie = 0.0;

    double pid = 0.0;
    double c = 0.0;
    double dt = 1.0/100.0;

    //Waiting for the first desired value
    while(!_first_des_value ) {
        usleep(0.1*1e6);
    }


    cout << "LOOP" << endl;

    myfile.open ("example.txt");
    myfile << "reference: " << _xdes << "\n";
    myfile.close();
    
    _xdes == _xmes; //Set the initial value

    //Neverending loop
    while( true ) {

        //PID errors
        e = _xdes - _xmes;
        de = e / dt;
        ie += e*dt;

        //PID action
        pid = _kp*e+ _kd*de + _ki*ie;

        //Output
        c += pid*dt;

        cout << "PID error: " << e << " output: " << c << endl;
        myfile.open ("example.txt", ios::app);
        myfile << "PID error: " << e << " output: " << c << "\n";
        myfile.close();

        if(abs(e)<_eps) break;

        usleep(10000);  //10000 microseconds = 0.01 seconds = 100Hz

        _xmes = c;      //System simulation: we assume that the output of the system is it's current value
    }
}