#include "control_loop.h"

//We can use the class constructor to set parameters
CONTROLLER::CONTROLLER(float kp_, float kd_, float ki_) {

    //Init
    _kp = kp_;
    _ki = ki_;
    _kd = kd_;

    _eps = 0.001;

    _first_des_value = false;

    CONTROLLER::system_start();
    
    //We can also run important thread of our system
    boost::thread loop_t( &CONTROLLER::loop, this);                //Main control loop
    
    //boost::thread NAME_THREAD( &CLASS_OF_THE_FUNCT::NAME_OF_THE_FUNCTION, CONTEXT, PARAMETERS )
}


//Sense: get input to change the state of our System
void CONTROLLER::set_xdes(double x) {
    _xdes = x;

    _first_des_value = true;
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
    double ep = 0.0;
    double de = 0.0;
    double ie = 0.0;

    double pid = 0.0;
    double c = 0.0;
    double dt = 1.0/100.0;

    //It's always convenient to wait the correct start of the system
    //To avoid to work with old (uninitialized values) 
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
        de = (e - ep) / dt;
        ie += e*dt;

        //PID action
        pid = _kp*e+ _kd*de + _ki*ie;

        //Output: control = control * time 
        c += pid*dt;

        cout << "System error: " << e << " System output: " << c << endl;
        myfile.open ("example.txt", ios::app);
        myfile << "System error: " << e << " System output: " << c << "\n";
        myfile.close();

        if(abs(e)<_eps) break;

        usleep(10000);  //10000 microseconds = 0.01 seconds = 100Hz

        _xmes = c;      //System simulation: we assume that the output of the system is it's current value
    }
    
    
}