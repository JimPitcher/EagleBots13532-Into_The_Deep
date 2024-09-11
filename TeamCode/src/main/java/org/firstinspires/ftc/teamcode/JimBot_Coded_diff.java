package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// This file is testing out driving the robot using calculated wheel speeds for turning while driving

@TeleOp(name = "Curved Steering OpMode")
public class JimBot_Coded_diff extends LinearOpMode {

    // Constants for robot geometry
    private static final double WHEELBASE = 13; // Wheelbase (distance between front and rear axles) in IN.
    private static final double TRACK_WIDTH = 14; // Track width (distance between left and right wheels) in IN.
    // Define the motors
    DcMotor FLMotor;
    DcMotor FRMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;

    Servo FLServo, BLServo, BRServo, FRServo;

    @Override
    public void runOpMode() {
        // Initialize hardware stuff
        initRobot();


        waitForStart(); // Waits for the Start button to be pressed
        while (opModeIsActive()) {

            // Gets the user inputs from the gamepad
            double drive = -gamepad1.left_stick_y; // Forward/backward movement
            double turn = gamepad1.right_stick_x;  // Left/right turn

            // Calculates the angular velocity and turning radius based on user inputs
            double turningRadius = calculateTurningRadius(turn); // Radius in IN.
            double angularVelocity = drive / turningRadius; // Calculate angular velocity (rad/s)

            // Calculates the speeds for each motor
            double[] wheelSpeed = calculateWheelSpeed(angularVelocity, turningRadius);
            double[] wheelAngle = calculateWheelAngles(turningRadius);

            moveIt(wheelSpeed, wheelAngle);

        }
    }


    public void initRobot() {

        // Maps where the motors are physically connected
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        // Reverse the right motors to account for their opposite orientation
        FLMotor.setDirection(FORWARD);
        FRMotor.setDirection(REVERSE);
        BLMotor.setDirection(FORWARD);
        BRMotor.setDirection(REVERSE);

        // Sets what happens when no power is applied to the motors.
        // In this mode, the computer will short the 2 leads of the motor, and because of math, the motor will be a lot harder to turn
        FLMotor.setZeroPowerBehavior(BRAKE);
        FRMotor.setZeroPowerBehavior(BRAKE);
        BLMotor.setZeroPowerBehavior(BRAKE);
        BRMotor.setZeroPowerBehavior(BRAKE);


        // Maps the servo objects to the physical ports
        FLServo = hardwareMap.get(Servo.class, "FLServo");
        FRServo = hardwareMap.get(Servo.class, "FRServo");
        BLServo = hardwareMap.get(Servo.class, "BLServo");
        BRServo = hardwareMap.get(Servo.class, "BRServo");

        // Sets the ends of the servos. Hover cursor over function for more info
        // Will need to be tuned later
        FLServo.scaleRange(0.0, 1.0);
        FRServo.scaleRange(0.0, 1.0);
        BLServo.scaleRange(0.0, 1.0);
        BRServo.scaleRange(0.0, 1.0);

    }


    public void moveIt(double wheelSpeed[], double wheelAngle[]) {

        if (wheelAngle[0] > 0) // Turning right
        {
            // Set power to motors based on calculated speeds
            FLMotor.setPower(wheelSpeed[0]); // Outside Front
            FRMotor.setPower(wheelSpeed[1]); // Inside Front
            BLMotor.setPower(wheelSpeed[2]); // Outside Rear
            BRMotor.setPower(wheelSpeed[3]); // Inside Rear
        } else if (wheelAngle[0] < 0) // Turning left
        {
            // Set power to motors based on calculated speeds
            FLMotor.setPower(wheelSpeed[0]); // Inside Front
            FRMotor.setPower(wheelSpeed[1]); // Outside Front
            BLMotor.setPower(wheelSpeed[2]); // Inside Rear
            BRMotor.setPower(wheelSpeed[3]); // Outside Rear
        }

        // Sets the custom positions of the wheels
        FLServo.setPosition(wheelAngle[0]); // Left
        FRServo.setPosition(wheelAngle[1]); // Right
        BLServo.setPosition(wheelAngle[0]); // Left
        BRServo.setPosition(wheelAngle[1]); // Right
    }


    // Function to calculate wheel speeds for a given angular velocity and turning radius
    private double[] calculateWheelSpeed(double angularVelocity, double turningRadius) {
        // Inside and outside turning radii for front/rear wheels
        double R_inner = turningRadius - (TRACK_WIDTH / 2);
        double R_outer = turningRadius + (TRACK_WIDTH / 2);

        // Calculate wheel speeds
        double V_IF = angularVelocity * R_inner; // Inside Front
        double V_OF = angularVelocity * R_outer; // Outside Front
        double V_IB = angularVelocity * R_inner; // Inside Rear
        double V_OB = angularVelocity * R_outer; // Outside Rear

        // If any wheel speeds are above 1.0, scales all the others down so that it is at max speed
        double maxSpeed = Math.max(Math.max(Math.abs(V_IF), Math.abs(V_OF)), Math.max(Math.abs(V_IB), Math.abs(V_OB)));
        if (maxSpeed > 1.0) {
            V_IF /= maxSpeed;
            V_OF /= maxSpeed;
            V_IB /= maxSpeed;
            V_OB /= maxSpeed;
        }

        return new double[]{V_IF, V_OF, V_IB, V_OB};
    }


    // Calculates the inner and outer wheel angles for a given radius
    public double[] calculateWheelAngles(double turningRadius) {

        double inner = Math.atan(WHEELBASE / (turningRadius - (TRACK_WIDTH / 2)));
        double outer = Math.atan(WHEELBASE / (turningRadius + (TRACK_WIDTH / 2)));

        return new double[]{inner, outer};
    }


    // Calculates the turning radius based on the turn input
    private double calculateTurningRadius(double turn) {
        // Converts joystick input (-1 to 1) to turning radius

        double maxTurnRadius = 24; // Max turning radius in IN., might need to be changed

        if (Math.abs(turn) < 0.01) {
            return Double.POSITIVE_INFINITY; // Moving straight
        }

        // Smaller turn input means tighter radius
        return maxTurnRadius / Math.abs(turn);
    }
}
