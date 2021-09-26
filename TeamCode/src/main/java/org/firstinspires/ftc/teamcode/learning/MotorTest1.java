package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Raca", group = "learning")
public class MotorTest1 extends LinearOpMode {

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor colector;
    DcMotor launcher;
    Servo thrower;
    DcMotor warm;
    Servo grab;




    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        grab = hardwareMap.get(Servo.class,"grab");

        grab.setDirection(Servo.Direction.FORWARD);



        thrower = hardwareMap.get(Servo.class,"thrower");

        thrower.setDirection(Servo.Direction.FORWARD);



        frontright = hardwareMap.get(DcMotor.class, "frontright");

        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        backleft = hardwareMap.get(DcMotor.class, "backleft");

        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        backright = hardwareMap.get(DcMotor.class, "backright");

        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        colector = hardwareMap.get(DcMotor.class, "colector");

        colector.setDirection(DcMotorSimple.Direction.FORWARD);
        colector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        colector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colector.setPower(0);


        launcher = hardwareMap.get(DcMotor.class, "launcher");

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        warm = hardwareMap.get(DcMotor.class, "warm");

        warm.setDirection(DcMotorSimple.Direction.REVERSE);
        warm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        warm.setTargetPosition(0);
        warm.setPower(0);
        warm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        thrower.setPosition(45/180.0);

        waitForStart();

        while(opModeIsActive()) {


            if (gamepad1.dpad_up && gamepad1.dpad_right){
                frontleft.setPower(1);
                frontright.setPower(0);
                backright.setPower(1);
                backleft.setPower(0);
            }

            else if (gamepad1.dpad_up && gamepad1.dpad_left){
                frontleft.setPower(0);
                frontright.setPower(1);
                backright.setPower(0);
                backleft.setPower(1);
            }


           else if (gamepad1.dpad_down && gamepad1.dpad_left){
                frontleft.setPower(-1);
                frontright.setPower(0);
                backright.setPower(-1);
                backleft.setPower(0);
            }

            else if (gamepad1.dpad_down && gamepad1.dpad_right){
                frontleft.setPower(0);
                frontright.setPower(-1);
                backright.setPower(0);
                backleft.setPower(-1);
            }





            else if (gamepad1.dpad_up) {
                frontleft.setPower(1);
                frontright.setPower(1);
                backright.setPower(1);
                backleft.setPower(1);
            }


            else if (gamepad1.dpad_down) {
                frontleft.setPower(-1);
                frontright.setPower(-1);
                backright.setPower(-1);
                backleft.setPower(-1);
            }


            else if (gamepad1.right_stick_x  < -0.2) {
                frontleft.setPower(-1);
                frontright.setPower(1);
                backright.setPower(1);
                backleft.setPower(-1);
            }



            else if (gamepad1.right_stick_x > 0.2) {
                frontleft.setPower(1);
                frontright.setPower(-1);
                backright.setPower(-1);
                backleft.setPower(1);
            }


            else if (gamepad1.dpad_right) {
                frontleft.setPower(1);
                frontright.setPower(-1);
                backright.setPower(1);
                backleft.setPower(-1);
            }

            else if (gamepad1.dpad_left) {
                frontleft.setPower(-1);
                frontright.setPower(1);
                backright.setPower(-1);
                backleft.setPower(1);
            }


            else {
                frontleft.setPower(0);
                frontright.setPower(0);
                backleft.setPower(0);
                backright.setPower(0);

            }

            if (gamepad1.a) {
                if (colector.getPower() == 0) {
                    colector.setPower(-1);
                    sleep(200);
                }
                else if (colector.getPower() == -1) {
                    colector.setPower(1);
                    sleep(200);
                }
                else if (colector.getPower() == 1) {
                    colector.setPower(0);
                    sleep(200);
                }
            }



            if (gamepad1.y) {
                if (launcher.getPower() == 0) {
                    launcher.setPower(0.65);
                    sleep(200);
                }
                else if (launcher.getPower() == 0.65) {
                    launcher.setPower(0);
                    sleep(200);
                }
            }

            if(colector.getPower()==1)
                launcher.setPower(0);
            else if(launcher.getPower()==1)
                colector.setPower(0);

            if (gamepad1.x) {
                thrower.setPosition(-0.05);

            }
            else
             thrower.setPosition(45/180.0);

            if(gamepad1.b) {
                warm.setPower(0.5);
                if(warm.getCurrentPosition()<20)
            warm.setTargetPosition(250);
                else if(warm.getCurrentPosition()>=250)
            warm.setTargetPosition(0);
            }

            if (gamepad1.left_bumper) {
                if (grab.getPosition() == 1)
                    grab.setPosition(0.8);
                else if (grab.getPosition() == 0.8)
                    grab.setPosition(1);
                sleep(200);
            }
        }
    }

}