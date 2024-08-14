#include "main.h"
#include "lemlib/api.hpp"
bool pneumatics = false;
bool intake = false;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({1, 2, 3}, pros::MotorGearset::green);
pros::MotorGroup right_motors({4, 5, 6}, pros::MotorGearset::green);
pros::Motor intake_motor(7, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_COUNTS);
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 8, lemlib::Omniwheel::NEW_325, 360, 2);
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
pros::ADIDigitalOut cylinder1('A', pneumatics);
pros::ADIDigitalOut cylinder2('B', pneumatics);
pros::Imu imu(10);
pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::OdomSensors sensors(&horizontal_tracking_wheel, nullptr , &vertical_tracking_wheel, nullptr, &imu);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();

}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {}

/**
 * Runs in driver control
 */
void opcontrol() {
    while (true) {
        int leftstickY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftstickX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        chassis.arcade(leftstickY, leftstickX);
        pros::delay(20);
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            pneumatics = !pneumatics;
            cylinder1.set_value(pneumatics);
            cylinder2.set_value(pneumatics);
            pros::delay(10);

        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake = !intake;
            if(intake) {
                intake_motor.move_velocity(200);
            }
        }
    }
}
