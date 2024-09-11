package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// This file is testing out driving the robot using calculated wheel speeds for turning while driving

@TeleOp(name = "Curved Steering OpMode")
public class JimBot_Coded_diff extends LinearOpMode {

    // Define the motors
    DcMotor FLMotor;
    DcMotor FRMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;

    // Constants for robot geometry
    private static final double L = 13; // Wheelbase (distance between front and rear axles) in IN.
    private static final double W = 14; // Track width (distance between left and right wheels) in IN.

    @Override
    public void runOpMode() {
        // Initialize hardware map
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        // Reverse the right motors to account for their opposite orientation
        FLMotor.setDirection(FORWARD);
        FRMotor.setDirection(REVERSE);
        BLMotor.setDirection(FORWARD);
        BRMotor.setDirection(REVERSE);


        waitForStart(); // Waits for the Start button to be pressed
        while (opModeIsActive()) {

            // Gets the user inputs from the gamepad
            double drive = -gamepad1.left_stick_y; // Forward/backward movement
            double turn = gamepad1.right_stick_x;  // Left/right turn

            // Calculates the angular velocity and turning radius based on user inputs
            double turningRadius = calculateTurningRadius(turn); // Radius in IN.
            double angularVelocity = drive / turningRadius; // Calculate angular velocity (rad/s)

            // Calculates the speeds for each motor
            double[] wheelSpeeds = calculateWheelSpeeds(angularVelocity, turningRadius);

            // Set power to motors based on calculated speeds
            FLMotor.setPower(wheelSpeeds[0]);  // Inside Front
            FRMotor.setPower(wheelSpeeds[1]); // Outside Front
            BLMotor.setPower(wheelSpeeds[2]);   // Inside Rear
            BRMotor.setPower(wheelSpeeds[3]);  // Outside Rear

            // Send telemetry data to the driver station for debugging
            telemetry.addData("Front Left Power", wheelSpeeds[0]);
            telemetry.addData("Front Right Power", wheelSpeeds[1]);
            telemetry.addData("Rear Left Power", wheelSpeeds[2]);
            telemetry.addData("Rear Right Power", wheelSpeeds[3]);
            telemetry.update();
        }
    }

    // Function to calculate wheel speeds for a given angular velocity and turning radius
    private double[] calculateWheelSpeeds(double angularVelocity, double turningRadius) {
        // Inside and outside turning radii for front/rear wheels
        double R_inner = turningRadius - W / 2;
        double R_outer = turningRadius + W / 2;

        // Calculate wheel velocities
        double V_IF = angularVelocity * R_inner; // Inside Front
        double V_OF = angularVelocity * R_outer; // Outside Front
        double V_IR = angularVelocity * R_inner; // Inside Rear
        double V_OR = angularVelocity * R_outer; // Outside Rear

        // Normalize wheel speeds to ensure they are within [-1, 1] for motor power
        double maxSpeed = Math.max(Math.max(Math.abs(V_IF), Math.abs(V_OF)), Math.max(Math.abs(V_IR), Math.abs(V_OR)));
        if (maxSpeed > 1.0) {
            V_IF /= maxSpeed;
            V_OF /= maxSpeed;
            V_IR /= maxSpeed;
            V_OR /= maxSpeed;
        }

        return new double[]{V_IF, V_OF, V_IR, V_OR};
    }

    // Function to calculate the turning radius based on the turn input
    private double calculateTurningRadius(double turnInput) {
        // Simple formula to convert joystick input (-1 to 1) to turning radius
        double maxTurnRadius = 2.0; // Max turning radius in meters (tweak as needed)
        if (Math.abs(turnInput) < 0.01) {
            return Double.POSITIVE_INFINITY; // Moving straight
        }
        return maxTurnRadius / Math.abs(turnInput); // Smaller turn input means tighter radius
    }
}
