package org.firstinspires.ftc.teamcode.main.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libraries.hardware.WheelMotors;
import org.firstinspires.ftc.teamcode.libraries.implementations.GeneralInitImpl;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name = "3 Rings Auto", group = "main")
public class Auton3Rings extends LinearOpMode {

    private Servo loadServo;

    private DcMotor launcherWheelMotor;
    private DcMotor liftMotor;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private List<DcMotor> driveMotors = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        loadServo = new GeneralInitImpl().initServo(hardwareMap, "loader", Servo.Direction.FORWARD);

        launcherWheelMotor = new GeneralInitImpl().initMotor(hardwareMap, "launcher", DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor = new GeneralInitImpl().initMotor(hardwareMap, "lift", DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(4500);
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveMotors = new WheelMotors().initWheels(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft = driveMotors.get(0); frontRight = driveMotors.get(1);
        backLeft = driveMotors.get(2); backRight = driveMotors.get(3);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Waiting for ", "start");
            telemetry.update();
        }

        if(opModeIsActive()){

            loadServo.setPosition(9/180.0);
            launcherWheelMotor.setPower(0.56);

            liftMotor.setPower(1);
            while(liftMotor.isBusy()) {
                liftMotor.setPower(1);
            }

            liftMotor.setPower(0);

            while(Math.abs(backLeft.getCurrentPosition()) <= 100){

                backRight.setPower(-0.7);
                frontRight.setPower(-0.7);

                frontLeft.setPower(0.7);
                backLeft.setPower(0.7);
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            for(int i = 0; i <= 6; i++) {
                loadServo.setPosition(33 / 180.0);
                sleep(800);
                loadServo.setPosition(7 / 180.0);
                sleep(500);
            }
            launcherWheelMotor.setPower(0);
            loadServo.setPosition(9/180.0);

            liftMotor.setTargetPosition(25);
            liftMotor.setPower(1);
            while(liftMotor.isBusy()){
                liftMotor.setPower(1);
            }
            liftMotor.setPower(0);

            while(Math.abs(backLeft.getCurrentPosition()) > 40){

                backRight.setPower(0.4);
                frontRight.setPower(0.4);

                frontLeft.setPower(-0.4);
                backLeft.setPower(-0.4);
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            while(Math.abs(backLeft.getCurrentPosition()) <= 4350){
                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }
    }
}
