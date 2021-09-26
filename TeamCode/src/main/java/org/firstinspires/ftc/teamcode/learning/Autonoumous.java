package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "nigga", group = "learning")
public class Autonoumous extends LinearOpMode {

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor launcher;
    Servo thrower;


    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setTargetPosition(0);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setPower(0);


        frontright = hardwareMap.get(DcMotor.class, "frontright");
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setTargetPosition(0);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setPower(0);


        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setTargetPosition(0);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setPower(0);


        backright = hardwareMap.get(DcMotor.class, "backright");
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setTargetPosition(0);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setPower(0);


        launcher = hardwareMap.get(DcMotor.class, "launcher");

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        thrower = hardwareMap.get(Servo.class, "thrower");

        thrower.setDirection(Servo.Direction.FORWARD);

        waitForStart();


        while (frontright.getCurrentPosition() < 3500 && frontleft.getCurrentPosition() < 3500 && backright.getCurrentPosition() < 3500 && backleft.getCurrentPosition() < 3500) {
            frontright.setTargetPosition(3500);
            frontleft.setTargetPosition(3500);
            backleft.setTargetPosition(3500);
            backright.setTargetPosition(3500);

            frontright.setPower(1);
            frontleft.setPower(1);
            backright.setPower(1);
            backleft.setPower(1);
        }
        thrower.setPosition(45 / 180.0);
while(thrower.getPosition()!=0) {
    launcher.setPower(0.75);
    sleep(5000);
    thrower.setPosition(-0.05);
    sleep(1000);
    thrower.setPosition(45 / 180.0);
    sleep(1000);
    thrower.setPosition(-0.05);
    sleep(1000);
    thrower.setPosition(45 / 180.0);
    sleep(1000);
    thrower.setPosition(-0.05);
    sleep(1000);
    thrower.setPosition(45 / 180.0);
    sleep(1000);
    thrower.setPosition(0);


}

        while (frontright.getCurrentPosition() < 4000 && frontleft.getCurrentPosition() < 4000 && backright.getCurrentPosition() < 4000 && backleft.getCurrentPosition() < 4000) {
            frontright.setTargetPosition(4000);
            frontleft.setTargetPosition(4000);
            backleft.setTargetPosition(4000);
            backright.setTargetPosition(4000);

            frontright.setPower(1);
            frontleft.setPower(1);
            backright.setPower(1);
            backleft.setPower(1);
        }
    }
}