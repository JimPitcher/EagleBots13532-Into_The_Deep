package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class clawSlideImport extends LinearOpMode {

    DcMotor slide = null;
    DcMotor pivot = null;

    int limitSlide, limitPivot;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        while (opModeIsActive()){
            setSlide(-gamepad2.right_stick_y);
            setPivot(-gamepad2.left_stick_y);
        }
    }

    public void initRobot() {
        DcMotor slide = hardwareMap.get(DcMotor.class, "slide");
        DcMotor pivot = hardwareMap.get(DcMotor.class, "pivot");

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setPower(.01);
        pivot.setPower(.01);

        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        limitSlide = 9999;
        limitPivot = 9999;
    }

    public void setSlide(double x) {
        double d = (slide.getCurrentPosition() < limitPivot && slide.getCurrentPosition() > -limitPivot) ? x:0.0;
        slide.setPower(d);
    }

    public void setPivot(double x) {
        double d = (pivot.getCurrentPosition() < limitPivot && pivot.getCurrentPosition() > -limitPivot) ? x:0.0;
        pivot.setPower(d);

    }
}