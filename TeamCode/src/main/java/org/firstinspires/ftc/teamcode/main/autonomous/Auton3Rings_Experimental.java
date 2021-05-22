package org.firstinspires.ftc.teamcode.main.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptRevSPARKMini;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.hardware.Gyroscope;
import org.firstinspires.ftc.teamcode.libraries.hardware.WheelMotors;
import org.firstinspires.ftc.teamcode.libraries.implementations.GeneralInitImpl;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name = "Wall 3 ring auton", group = "test")

public class Auton3Rings_Experimental extends LinearOpMode {

    private BNO055IMU imu;

    private Servo loadServo;

    private DcMotorEx launcherWheelMotor;
    private DcMotor liftMotor;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private List<DcMotor> driveMotors = new ArrayList<>();

    float refGyro;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = new Gyroscope().initIMU(hardwareMap);

        refGyro = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle;

        loadServo = new GeneralInitImpl().initServo(hardwareMap, "loader", Servo.Direction.FORWARD);

        launcherWheelMotor = new GeneralInitImpl().initExMotor(hardwareMap, "launcher", DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherWheelMotor.setVelocityPIDFCoefficients(700, 0.7,130, 5);

        liftMotor = new GeneralInitImpl().initMotor(hardwareMap, "lift", DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(4345);
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
            refGyro = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Start pos gyro ", refGyro);
            telemetry.addData("Waiting for ", "start");
            telemetry.update();
        }

        if(opModeIsActive()){

            loadServo.setPosition(9/180.0);

            liftMotor.setPower(1);
            while(liftMotor.isBusy()) {
                liftMotor.setPower(1);
            }

            liftMotor.setPower(0);

            //move

            while(overallWheelEnc() <= 710 && !isStopRequested()) {
                telemetry.addData("Front right ", frontRight.getPower());
                telemetry.addData("Front left ", frontLeft.getPower());
                telemetry.addData("Back right ", backRight.getPower());
                telemetry.addData("Back left ", backLeft.getPower());
                telemetry.addData("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", "");
                sideways(0.5);
            }

            wheelStop();

            front(0.4, 3600);

            wheelStop();

            while(overallWheelEnc() <= 100 && !isStopRequested()) {
                telemetry.addData("Front right ", frontRight.getPower());
                telemetry.addData("Front left ", frontLeft.getPower());
                telemetry.addData("Back right ", backRight.getPower());
                telemetry.addData("Back left ", backLeft.getPower());
                telemetry.addData("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", "");
                sideways(0.5);
            }

            wheelStop();

            launcherWheelMotor.setVelocity(1500);
            sleep(1000);

            for(int i = 0; i <= 6; i++) {
                loadServo.setPosition(37 / 180.0);
                sleep(800);
                loadServo.setPosition(9 / 180.0);
                sleep(500);
            }
            launcherWheelMotor.setVelocity(0);
            loadServo.setPosition(9/180.0);

            liftMotor.setTargetPosition(25);
            liftMotor.setPower(1);
            while(liftMotor.isBusy()){
                liftMotor.setPower(1);
            }
            liftMotor.setPower(0);

            //move

            front(0.7, 700);

            wheelStop();

        }
    }

    class Correction{

        private double kp = 0;
        private double ki = 0;
        private double kd = 0;

        private float gyroError = 0;
        private float gyroError2 = 0;

        private double gyroCorrection(){

            float currentGyro = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle;
            gyroError = currentGyro - refGyro;

            telemetry.addData("Y axis rotation ", currentGyro);
            telemetry.addData("Initial Y axis rotation ", refGyro);
            telemetry.update();

            double speedChange = kp*gyroError + ki*(gyroError + gyroError2) + kd*(gyroError - gyroError2);

            gyroError2 = gyroError;

            return speedChange;
        }
    }

    private void sideways(double power) {

        double speedChange = new Correction().gyroCorrection();

        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power + speedChange);
        frontLeft.setPower(power - speedChange);
    }

    private void front(double power, long target){

        while (((Math.abs(overallWheelEnc())) <= Math.abs(target) || target == 0) && !isStopRequested()) {

            telemetry.addData("Front right ", frontRight.getPower());
            telemetry.addData("Front left ", frontLeft.getPower());
            telemetry.addData("Back right ", backRight.getPower());
            telemetry.addData("Back left ", backLeft.getPower());
            telemetry.addData("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", "");

            double speedchange = new Correction().gyroCorrection();

            frontRight.setPower(power);
            backRight.setPower(power);

            frontLeft.setPower(power - speedchange);
            backLeft.setPower(power - speedchange);
        }
    }

    private double overallWheelEnc(){
        double average =
                Math.abs(frontLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition())
                        + Math.abs(backLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition());
        return average/4;
    }

    private void resetWheelEnoders(){

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void wheelStop(){
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        resetWheelEnoders();
    }

}
