#include "main.h"

pros::Motor FrontRight1(10, pros::E_MOTOR_GEARSET_06 , false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FrontRight2(9, pros::E_MOTOR_GEARSET_06 , true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FrontLeft1(2, pros::E_MOTOR_GEARSET_06 , false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FrontLeftt2(3, pros::E_MOTOR_GEARSET_06 , true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BackRight1(19, pros::E_MOTOR_GEARSET_06 , true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BackRight2(18, pros::E_MOTOR_GEARSET_06 , false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BackLeft1(12, pros::E_MOTOR_GEARSET_06 , false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BackLeft2(13, pros::E_MOTOR_GEARSET_06 , true,pros::E_MOTOR_ENCODER_COUNTS);

pros::Rotation Right(7);
pros::Rotation Left(1);
pros::Rotation Back(5);

pros::Imu Inertial(17);

pros::Controller CONTROLLER(pros::E_CONTROLLER_MASTER);

