package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class JimBot extends LinearOpMode {

    DcMotor FLMotor, BLMotor, BRMotor, FRMotor;

    Servo FLServo, BLServo, BRServo, FRServo;

    ElapsedTime turnTime = new ElapsedTime();


    @Override
    public void runOpMode() {

        initRobot(); // Does all the robot stuff

        waitForStart();
        while (opModeIsActive()) {
            double speed = gamepad1.right_trigger + (-gamepad1.left_trigger); // Makes it so that the triggers cancel each other out if both are pulled at the same time
            double angle = gamepad1.right_stick_x;
            if (speed != 0)
                move(gamepad1.left_stick_x, speed);
            else if (angle != 0)
                rotate(angle);
            else {
                FRMotor.setPower(0);
                BLMotor.setPower(0);
                BRMotor.setPower(0);
                FRMotor.setPower(0);
            }

        }

    }


    public void initRobot() {

        // Maps the motor objects to the physical ports
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");

        // Sets the encoder mode
        FLMotor.setMode(RUN_USING_ENCODER);
        BLMotor.setMode(RUN_USING_ENCODER);
        BRMotor.setMode(RUN_USING_ENCODER);
        FRMotor.setMode(RUN_USING_ENCODER);

        // Sets what happens when no power is applied to the motors.
        // In this mode, the computer will short the 2 leads of the motor, and because of math, the motor will be a lot harder to turn
        FLMotor.setZeroPowerBehavior(BRAKE);
        BLMotor.setZeroPowerBehavior(BRAKE);
        BRMotor.setZeroPowerBehavior(BRAKE);
        FRMotor.setZeroPowerBehavior(BRAKE);

        FLMotor.setDirection(FORWARD);
        BLMotor.setDirection(FORWARD);
        BRMotor.setDirection(REVERSE);
        FRMotor.setDirection(REVERSE);


        // Maps the servo objects to the physical ports
        FLServo = hardwareMap.get(Servo.class, "FLServo");
        BLServo = hardwareMap.get(Servo.class, "BLServo");
        BRServo = hardwareMap.get(Servo.class, "BRServo");
        FRServo = hardwareMap.get(Servo.class, "FRServo");

        // Sets the ends of the servos. Hover cursor over function for more info
        // Will need to be tuned later
        FLServo.scaleRange(0.0, 1.0);
        BLServo.scaleRange(0.0, 1.0);
        BRServo.scaleRange(0.0, 1.0);
        FRServo.scaleRange(0.0, 1.0);

    }


    /*
    Converts cartesian coordinates to polar
    0 = r
    1 = theta
    */
    public double[] cartesianToPolar(double x, double y) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // Radius
        arrayToReturn[1] = Math.atan(y / x) * (Math.PI / 180); // Theta


        return arrayToReturn;
    }


    /*
    Converts polar coordinates to cartesian
     0 = x
     1 = y
    */
    public double[] polarToCartesian(double r, double theta) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = r * Math.cos(theta); // X
        arrayToReturn[1] = r * Math.sin(theta); // Y

        return arrayToReturn;
    }


    // Moves the robot in a straight line using heading power
    public void move(double heading, double power) {

        heading = (heading + 1) / 2;

        FLServo.setPosition(heading);
        BLServo.setPosition(heading);
        BRServo.setPosition(heading);
        FRServo.setPosition(heading);


        FLMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FRMotor.setPower(power);
    }

    // Rotates the robot around the center point
    public void rotate(double angle) {

        //set wheels for rotation
        FLServo.setPosition(.25);
        BLServo.setPosition(.75);
        BRServo.setPosition(.25);
        FRServo.setPosition(.75);

        //turn motors to rotate robot
        FLMotor.setPower(angle);
        BLMotor.setPower(angle);
        BRMotor.setPower(angle);
        FRMotor.setPower(angle);
    }


    double WHEEL_BASE = 16; // Front to back distance of the wheels
    double TRACK_WIDTH = 16; // Side to side distance of the wheels
    double WHEEL_RADIUS = 3;

    // Rotates while going along a curve similar to how a car works
    // Larger turning Radius is smaller turn amount
    // Positive radius is turning right
    // Negative radius is turning right
    public void rotateAlongCurve(double velocity, double turningRadius) {

        // Calculates the turning radius of the inside and outside wheels from the turning radius of the center
        double innerRadius = turningRadius - (TRACK_WIDTH / 2);
        double outerRadius = turningRadius + (TRACK_WIDTH / 2);

        // Calculates the different turning radius for the inner and outer sets of wheels
        double IAngle = Math.atan(WHEEL_BASE / innerRadius);
        double OAngle = Math.atan(WHEEL_BASE / outerRadius);

        // Calculates the power for the inner and outer sets of wheels
        double IPower = velocity * innerRadius;
        double OPower = velocity * outerRadius;

        // Turning one way needs the opposite calculations as turning the other. This just reverses that
        if (turningRadius > 0) // Turning Right
        {
            FLServo.setPosition(OAngle);
            BLServo.setPosition(OAngle);
            BRServo.setPosition(IAngle);
            FRServo.setPosition(IAngle);

            FLMotor.setPower(OPower);
            BLMotor.setPower(OPower);
            BRMotor.setPower(IPower);
            FRMotor.setPower(IPower);
        } else // Turning Left
        {
            FLServo.setPosition(IAngle);
            BLServo.setPosition(IAngle);
            BRServo.setPosition(OAngle);
            FRServo.setPosition(OAngle);

            FLMotor.setPower(IPower);
            BLMotor.setPower(IPower);
            BRMotor.setPower(OPower);
            FRMotor.setPower(OPower);
        }


    }

}
