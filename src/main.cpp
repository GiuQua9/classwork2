/*#include "control_loop.h"

using namespace std;

//the goal is to implement a simple PID controller, 
//running on input value to reach the desired one

// Sense: read a value from keyboard
// Plan:  generate the correct input
// Act:   set the input

int main(int argc, char** argv) {


    return 0;
}
*/

#include "control_loop.h"

using namespace std;

//the goal is to implement a simple PID controller, 
//running on input value to reach the desired one

// Sense: read a value from keyboard
// Plan:  generate the correct input
// Act:   set the input

int main(int argc, char** argv) {

    //New controller with the given gains for the PID
    CONTROLLER c(0.1,0.05,0.01);

    double x;
    while( true ) {
        cout << "Insert desired value for the controller: ";
        cin >> x;
        c.set_xdes(x);
    }
    
    //What happen without the while loop?
    //Should I wait after the input?

    return 0;
}