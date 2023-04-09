#include "main.h"
#include "math.h"


#define close_turn 90
#define auton 5
using namespace std;

// GLOBAL POSITION VARIABLES
double x = 0;
double y = 0;
double theta = 0; 
double boundedTheta = 0; //theta that exists from [0, 2pi)

//PHYSICAL CONSTANTS
double wheelRadius = 3.25/2; //radius of omniwheels
double rightRadius = 8.4719; //distance from tracking center from right wheel in inches
double leftRadius = 8.4587; //distance from tracking center from left wheel in inches
double backRadius = 8.5391; //distance from tracking center from back (perpendicular) wheel in inches
double radians = M_PI/(100*180); //centidegrees to radians 

//PREVIOUS ENCODER POSITIONS
double prevL = 0;
double prevR = 0;
double prevB = 0;
double prevT = 0;

//CHANGE IN ENCODER POSITIONS
double DeltaL = 0;
double DeltaR = 0;
double DeltaB = 0;
double DeltaT = 0;

//ODOM DISPLACEMENT VARS
double localDispX = 0;
double localDispY = 0;

// /*
//  * Init the autonomous tracking
//  * Run at the beginning of any autonomous routine
//  */
void odomInit() {
    Right.reset_position();
    Left.reset_position();
    Back.reset_position();

    Left.set_reversed(true);
    Back.set_reversed(true);
}
 
//LV_IMG_DECLARE(rushia);
void initialize() {
    // lv_obj_t * img_src = lv_img_create(lv_scr_act(), NULL); /*Crate an image object*/
    // lv_img_set_src(img_src, &rushia);  /*Set the created file as image*/
    // lv_obj_set_pos(img_src, 0, 0);      /*Set the positions*/ 
    // lv_obj_set_drag(img_src, true);
    
    pros::lcd::initialize();
    
    Inertial.reset();
    pros::delay(3000);
    odomInit();

    
    pros::Task odom(Track, TASK_PRIORITY_MAX);
    pros::Task tankdrive(Tank);
}



/*
 * @param x - the untransformed displacement vector
 * @param theta - current IMU reading for the bot IN RADIANS
 * @return - pair of doubles of the transformed displacements x, y
 */

static pair<double, double> Transform(double x, double theta) {
    double x2 = (x * cos(theta));
    double y2 = (x * sin(theta));
    return make_pair(x2, y2);
}





void tuneTracker() {
        double expected = (2*M_PI*leftRadius)*theta/(2*M_PI);
        double actual = (Left.get_position()*wheelRadius*radians);
        pros::lcd::set_text(1, "expected left: " + to_string(expected));
        pros::lcd::set_text(2, "actual left: " + to_string(actual));
        double expected1 = (2*M_PI*rightRadius)*theta/(2*M_PI);
        double actual1 = (Right.get_position()*wheelRadius*radians);
        pros::lcd::set_text(3, "expected right: " + to_string(expected1));
        pros::lcd::set_text(4, "actual right: " + to_string(actual1));
        double expected2 = (2*M_PI*backRadius)*theta/(2*M_PI);
        double actual2 = (Back.get_position()*wheelRadius*radians);
        pros::lcd::set_text(5, "expected back: " + to_string(expected2));
        pros::lcd::set_text(6, "actual back: " + to_string(actual2));

}



void Track() {
    while (true) {
        //calculate the angle of the robot as an absolute quantity based on encoder positions
        //theta = ((Left.get_position()*wheelRadius*radians) - (Right.get_position()*wheelRadius*radians))/(leftRadius+rightRadius);
        theta = Inertial.get_rotation()*M_PI/180;

        //calculate change in values relative to previous position
        DeltaT = theta-prevT;
        //DeltaL = (Left.get_position()*wheelRadius*radians) - prevL;
        DeltaR = (Right.get_position()*wheelRadius*radians) - prevR;
        DeltaB = (Back.get_position()*wheelRadius*radians) - prevB;

        //calcuate a theta that exists on [0,2pi)
        boundedTheta = theta; 
        while (boundedTheta < 0) boundedTheta +=2*M_PI;
        boundedTheta = fmod(theta, 2*M_PI);
        
        //if there is no change in angle
        if(DeltaT == 0) { 
            //the displacement the robot traveled must be r*sin(theta) and r*cos(theta)
            //add this displacement to the global position variables
            x += DeltaR * sin (theta);
            y += DeltaR * cos (theta);
        } else {
            //calculate how much the robot has moved, with +y being defined as the robot's front
            //assumes the robot has moved in a straight line, so any arcs are approximated by a chord of a circle with length 2*r*sin(theta/2) 
            localDispX = 2*sin(-DeltaT/2)*(((DeltaB)/DeltaT)+backRadius);
            localDispY = 2*sin(DeltaT/2)*(((DeltaR)/DeltaT)+rightRadius);
            
            //rotate this chord back by theta so that +y is now defined to be where the robot started from
            //add this displacement to the global position variables
            x += cos(-theta-DeltaT/2)*localDispX - sin(-theta-DeltaT/2)*localDispY;
            y += sin(-theta-DeltaT/2)*localDispX + cos(-theta-DeltaT/2)*localDispY;
        }
        
        pros::lcd::set_text(6, "Coordinates: " + to_string(x) + ", " + to_string(y));
        pros::lcd::set_text(7, "Angle: " + to_string(theta*180/M_PI));
        
        //store the previous iteration's values
        //prevL = (Left.get_position()*wheelRadius*radians);
        prevR = (Right.get_position()*wheelRadius*radians);
        prevB = (Back.get_position()*wheelRadius*radians);
        prevT = theta;
        pros::delay(10);
    }
}
       


//set power to each of the chassis motors
void Chassis(double FR, double BR, double FL, double BL) {
    FrontRight1.move(FR);
    FrontRight2.move(FR);
    FrontLeft1.move(FL);
    FrontLeftt2.move(FL);
    BackRight1.move(BR);
    BackRight2.move(BR);
    BackLeft1.move(BL);
    BackLeft2.move(BL);
}

void Tank() {
    // int c = 1;
    // while (true) {
    //     Chassis(FrontRight1.get_actual_velocity()-Back.get_position()*wheelRadius*radians*c
    //             ,BackRight1.get_actual_velocity()-Back.get_position()*wheelRadius*radians*c
    //             ,FrontLeft1.get_actual_velocity()+Back.get_position()*wheelRadius*radians*c
    //             ,BackLeft1.get_actual_velocity()+Back.get_position()*wheelRadius*radians*c);
    // }
}

void turn(double turn, double tolerance) {

    double targetAngle = fmod(turn, 360.0);
    double currentAngle = boundedTheta*180/M_PI;
    double turnDifference = targetAngle - currentAngle;

    double prevDifference = turnDifference;
    double derivative = 127;
    double integral = 0;

    double kP = 1.25;
    double kI = 3.8; //3.8
    double kD = 0.33;

    double seconds = 0.015; // 10 ms to s

    int count = 0;

    bool exitCondition = false;
    int exitConditionCount = 0;
    while (exitConditionCount < 6) {
        exitCondition = !(abs(turnDifference) > tolerance || abs(derivative) > 0.1);
        if (exitCondition == true) exitConditionCount++;
        else exitConditionCount = 0;
        currentAngle = theta*180/M_PI;
        turnDifference = targetAngle-currentAngle;

        // Spins in the directions which will allow bot to complete turn fastest
        if (turnDifference > 180)
            turnDifference = 360 - turnDifference;


        double pid = kP*(turnDifference) + kI*(integral) + kD*derivative; 
        Chassis(pid, pid, pid, pid);
        pros::delay(seconds*1000);
        derivative = (turnDifference-prevDifference)/seconds;
        integral += turnDifference*seconds;
        if (turnDifference*prevDifference < 0) integral  =0;
        prevDifference=turnDifference;
        count++;
        if (count %10 == 0) CONTROLLER.set_text(0,0,std::to_string(turnDifference));


    }
    Chassis(0,0,0,0);
    
}

void MTRP(double x1, double y1, double turn) {



    double targetDisp = sqrt((x1-x)*(x1-x) + (y1-y)*(y1-y)); // uses the distance formula to create the robots target displacement
    double currentDisp = 0;// sets the robots initial displacement as 0
    double dispDifference = targetDisp;// since the robots initial displacement is 0, the displacement difference is equal to the target displacement
    double prevDisp = dispDifference;// change in displacement is always equal to displacement difference, so 10 ms previously can be set to displacement difference
    double targetTheta = atan((y1-y)/(x1-x));// tangent inverse of the new displacement to find the target angle
    if (x1-x < 0) targetTheta += M_PI; // arctan is only defined on the positive x axis, so add pi if x1-x is negative x axis
    targetTheta = fmod(targetTheta+2*M_PI, 2*M_PI);// bounds theta on 0 to 2pi

    double dispDerivative = 0;//robot isnt moving so dispDerivative is 0 
    double dispIntegral = 0;//robots position isnt changing so initial condition is 0, so dispIntegral is 0

    double kPdisp = 0.6;// proportional constant, adds power proportional to the error
    double kIdisp = 0.35;// more power to make the area under the displacment curve decrease to 0
    double kDdisp = 0.20;// dampens the acceleration of the robot to maintain control over robots movement



    double targetAngle = fmod(turn, 360.0);// bounds target angle within 0 and 360 degrees
    double currentAngle = boundedTheta*180/M_PI;//converts boundedtheta from radians to degrees
    double turnDifference = targetAngle - currentAngle;//change in the turn is the difference between the desired angle and the current angle

    double prevDifference = turnDifference;//turn difference is equal to previous difference because the change in angle is always the same
    double turnDerivative = 0;//robot is not turning
    double turnIntegral = 0;//robot starts at an angle of 0

    double kPturn = 1.15;//power allocated to the robot is proportional to the error, power to correct this error
    double kIturn = 1.0;// more power means that the area under the rotational displacement curve reduces to 0
    double kDturn = 0.12;//dampens the rotational acceleration of the robot to control turning movement

    double seconds = 0.01; // 10 ms to s


    bool exitCondition = false; //exitcondition begins as false and needs to switch to true to exit the while loop
    int exitConditionCount = 0;//exit condition count begins as 0
    while (exitConditionCount < 6) { //robot remains in the loop while exit condition count is less than 6, exits the loop when the count reaches 6
        exitCondition = abs(dispDerivative) < 0.2 && abs(turnDerivative) < 0.2 && abs(dispDifference) < 1 && abs(turnDifference) < 2;// checks whether or not robot is within the set amount of error from the target
        if (exitCondition) exitConditionCount++;// if the above conditions are satisfied then exit condition count increases by 1
        else exitConditionCount = 0;// if the above conditions are broken at any time then exit condition count resets

        //update angle error, finds the error at current rotational displacement rather than original displacement
        currentAngle = boundedTheta*180/M_PI;// current angle is the bounded theta converted to degrees
        turnDifference = targetAngle-currentAngle;// difference in the turn is the difference between the desired angle and the current angle

        // Spins in the directions which will allow bot to complete turn fastest
        if (turnDifference > 180)//condition is that the change in angle is more than 180
            turnDifference = 360 - turnDifference;// if so, switches the direction the turn goes in

        //update displacement error, finds the error at the current position rather than original position
        dispDifference = sqrt((x1-x)*(x1-x) + (y1-y)*(y1-y));//
        targetTheta = atan((y1-y)/(x1-x));
        if (x1-x < 0) targetTheta += M_PI;
        targetTheta = fmod(targetTheta+2*M_PI, 2*M_PI);

        // if (abs(dispDifference) < 1.1) kPdisp = 0.8;
        // else kPdisp = 0.7;
        //wants to decrease area under rotational displacement curve with more power when turn difference is less than 4
        if (abs(turnDifference) < 4) kIturn = 1.60;
        else kIturn = 1.0;


        double turnPid = kPturn*(turnDifference) + kIturn*(turnIntegral) + kDturn*turnDerivative; 
        double dispPid = kPdisp*(dispDifference) + kIdisp*(dispIntegral) + kDdisp*dispDerivative;

        double finalX = (dispDifference*cos(targetTheta)*cos(-theta)) + (dispDifference*sin(targetTheta)*sin(-theta));
        double finalY = (dispDifference*cos(targetTheta)*sin(theta)) + (dispDifference*sin(targetTheta)*cos(theta));

        Chassis(dispPid*(finalX-finalY) + turnPid, dispPid*(-finalX-finalY) + turnPid, dispPid*(finalX+finalY) + turnPid, dispPid*(-finalX+finalY) + turnPid);
        pros::delay(seconds*1000);
        if (prevDifference * turnDifference < 0) turnIntegral = 0;
        if (dispDifference * prevDisp < 0) dispIntegral = 0;
        turnDerivative = (turnDifference-prevDifference)/seconds;
        turnIntegral += turnDifference*seconds;
        dispDerivative = (dispDifference-prevDisp)/seconds;
        dispIntegral += dispDifference*seconds;

        prevDifference=turnDifference;
        prevDisp=dispDifference;
        pros::lcd::set_text(1, "Target T: " + to_string(targetTheta));

    }
    Chassis(0,0,0,0);


}

// void PP(double x, double y, double theta, double a) {
//     double currentY = y;
//     double lookAheadY = log(1/(currentY+a) + sqrt((1/((currentY+a)(currentY+a)) -1)));// lookAhead = log(1/x + sqrt((1/(x*x)) -1));
//     double lookAheadX = a;
  
// }

void competition_initialize() { initialize(); }


void autonomous() {
    //MTRP(-0, 24 , 90);
    MTRP(-0, -44 ,90);
    MTRP (-18, -44, 180);
    MTRP (-18, -6, 270);
    MTRP (-0, -6, 0);
}
