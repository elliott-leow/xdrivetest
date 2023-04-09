#include "main.h"
#include "math.h"
using namespace pros;
using namespace std;

double speed = 1;
int ct =0;
bool fieldCentric = true;

static pair<double, double> rotate(pair<double, double> point, double theta) {
    return make_pair(cos(theta)*point.first - sin(theta)*point.second, sin(theta)*point.first + cos(theta)*point.second);
}

//projection of b (p1) on to a (p2)
pair<double,double> proj(pair<double, double> p1,  pair<double, double> p2) {
    double scalar = (p1.first*p2.first+p1.second*p2.second)/(p2.first*p2.first+p2.second*p2.second);
    return make_pair(p2.first*scalar, p2.second*scalar);

}

double mag(pair<double, double> vector) {
    return sqrt(vector.first*vector.first+vector.second*vector.second);
}

void opcontrol() {
    while (true) {
        


        if (CONTROLLER.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
            speed == 1 ? speed = 0.3 : speed = 1;
        } 


        double analogLeftX = CONTROLLER.get_analog(ANALOG_LEFT_X) * speed; 
        double analogLeftY = CONTROLLER.get_analog(ANALOG_LEFT_Y) * speed; 
        double analogRightX = CONTROLLER.get_analog(ANALOG_RIGHT_X) * speed;

        
        pair<double, double> controlVector = make_pair(analogLeftX, analogLeftY);
        

        if (fieldCentric == true) {
            //assume motor spins counterclockwisedouble 
            double finalX = (analogLeftX * cos(-theta)) + (analogLeftY * sin(-theta));
            double finalY = (analogLeftX * sin(theta)) + (analogLeftY * cos(theta));
            analogLeftX = finalX;
	        analogLeftY = finalY;
		} 
        double FL = analogRightX + analogLeftY + analogLeftX;
        double FR = analogRightX - analogLeftY + analogLeftX;
        double BL = analogRightX + analogLeftY - analogLeftX;
        double BR = analogRightX - analogLeftY - analogLeftX;
        FrontRight1.move(FR);
        FrontRight2.move(FR);
        FrontLeft1.move(FL);
        FrontLeftt2.move(FL);
        BackRight1.move(BR);
        BackRight2.move(BR);
        BackLeft1.move(BL);
        BackLeft2.move(BL);
        //Track();
        //tuneTracker();
        pros::delay(20);
        ct++;
        //if (count % 10 == 0) CONTROLLER.set_text(0,0,std::to_string(theta));

    }
}
