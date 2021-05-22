package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libraries.hardware.Gyroscope;
import org.firstinspires.ftc.teamcode.libraries.hardware.WheelMotors;
import org.firstinspires.ftc.teamcode.libraries.implementations.GeneralInitImpl;

import java.util.*;

@Disabled
@TeleOp(name = "New Driving", group = "test")

public class test_driving extends LinearOpMode {


    int viteza = 0;
    double power = 0.8;
    double halfPower = 0.5;

    double put, ung;

    ElapsedTime runTime = new ElapsedTime();

    static final double INITIAL_ANGLE = 45;

    static final String COLLECTOR_MOTOR = "collector";
    static final String LAUNCHER_MOTOR = "launcher";
    static final String TURRET_MOTOR = "turret";
    static final String LIFT_MOTOR = "lift";
    static final String LOADING_SERVO = "loader";

    private GeneralInitImpl init = new GeneralInitImpl();

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    List<DcMotor> driveMotors = new ArrayList<>();

    private DcMotor liftMotor, turretMotor, collectorMotor;
    private DcMotorEx launcherWheelMotor;

    private Servo loadServo;

    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = new Gyroscope().initIMU(hardwareMap);

        turretMotor = init.initMotor(hardwareMap,
                TURRET_MOTOR,
                DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherWheelMotor = init.initExMotor(hardwareMap,
                LAUNCHER_MOTOR,
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER);
        launcherWheelMotor.setVelocityPIDFCoefficients(700, 0.7,130, 5);

        loadServo = init.initServo(hardwareMap,
                LOADING_SERVO,
                Servo.Direction.FORWARD);
        loadServo.setPosition(INITIAL_ANGLE/180.0);

        collectorMotor = init.initMotor(hardwareMap,
                COLLECTOR_MOTOR,
                DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveMotors = new WheelMotors().initWheels(hardwareMap,
                DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft = driveMotors.get(0); frontRight = driveMotors.get(1);
        backLeft = driveMotors.get(2); backRight = driveMotors.get(3);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Waiting for start", "");
            telemetry.update();
        }

        while(opModeIsActive()) {

            telemetry.addData("~~~~~~~~~~~~~~~~~~~~~~~~", "");

            smoothMovement(false); /* uses gamepad1: dpad, right stick */
            launcherRun(true); /* uses gamepad1: right trigger */
            loadRing(false); /* uses gamepad1: right bumper */
            collectorRun(false); /* uses gamepad1: a */

            telemetry.update();

        }
    }

    public void launcherRun(boolean getLogs){

        if(gamepad1.right_trigger >= 0.7 && launcherWheelMotor.getPower() == 0){
            //launcherWheelMotor.setVelocity(700); //1500 driving
            launcherWheelMotor.setPower(0.4);
            sleep(200);
        }
        else if(gamepad1.right_trigger >= 0.7 && launcherWheelMotor.getPower() != 0){
            //launcherWheelMotor.setVelocity(0);
            launcherWheelMotor.setPower(0);
            sleep(200);
        }

        if(getLogs){
            telemetry.addData("Launcher velocity ", launcherWheelMotor.getVelocity());
        }
    }

    public void loadRing(boolean getLogs){

        if(gamepad1.x && loadServo.getPosition() > 40/180.0){
            runTime.reset();
            loadServo.setPosition(25/180.0);
            sleep(200);
        }
        else if (loadServo.getPosition() < 40/180.0 && runTime.milliseconds() > 350){
            loadServo.setPosition(INITIAL_ANGLE/180.0);
        }

        if(getLogs){
            telemetry.addData("Load servo position ", loadServo.getPosition());
        }
    }

    public void collectorRun(boolean getLogs){

        if(gamepad1.a && collectorMotor.getPower() == 0){
            collectorMotor.setPower(1);
            sleep(200);
        }
        else if(gamepad1.a && collectorMotor.getPower() == 1){
            collectorMotor.setPower(-1);
            sleep(200);
        }
        else if(gamepad1.a && collectorMotor.getPower() != 0){
            collectorMotor.setPower(0);
            sleep(200);
        }

        if(getLogs){
            telemetry.addData("Collector power ", collectorMotor.getPower());
        }
    }

    public void smoothMovement(boolean getLogs) throws InterruptedException {

        if(getLogs){
            telemetry.addData("~~~~~~~~~~~~~~~~~~~~", "");
            telemetry.addData("Front left ", frontLeft.getCurrentPosition());
            telemetry.addData("Front right ", frontRight.getCurrentPosition());
            telemetry.addData("Back left ", backLeft.getCurrentPosition());
            telemetry.addData("Back right ", backRight.getCurrentPosition());
            telemetry.addData("~~~~~~~~~~~~~~~~~~~~", "");
        }

        ung = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y)*180/Math.PI;
        put = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y);
        if (ung<0)
            ung+=360;
        telemetry.addData("Unghi: ", ung);
        telemetry.addData("Putere: ", put);
        //rotire
        if (put==0)
        {
            if (gamepad1.right_stick_x < 0) {
                frontRight.setPower(Math.abs(gamepad1.right_stick_x));
                backRight.setPower(Math.abs(gamepad1.right_stick_x));
                frontLeft.setPower(gamepad1.right_stick_x);
                backLeft.setPower(gamepad1.right_stick_x);
            } else if (gamepad1.right_stick_x >= 0) {
                frontRight.setPower(-gamepad1.right_stick_x);
                backRight.setPower(-gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x);
                backLeft.setPower(gamepad1.right_stick_x);
            }
        }

        else
        {
            if (ung>=337.5 || ung<=22.5) //sus
            {
                frontLeft.setPower(put);
                frontRight.setPower(put);
                backRight.setPower(put);
                backLeft.setPower(put);
            }

            else if (ung>22.5 && ung<67.5) //diagonala dreapta sus
            {
                frontLeft.setPower(put);
                frontRight.setPower(0);
                backRight.setPower(put);
                backLeft.setPower(0);
            }

            else if (ung>=67.5 && ung<=112.5) //dreapta
            {
                frontLeft.setPower(put);
                frontRight.setPower(-put);
                backRight.setPower(put);
                backLeft.setPower(-put);
            }

            else if (ung>112.5 && ung<157.5) //diagonala dreapta jos
            {
                frontRight.setPower(-put);
                frontLeft.setPower(0);
                backLeft.setPower(-put);
                backRight.setPower(0);
            }

            else if (ung>=157.5 && ung<=202.5) //jos
            {
                frontLeft.setPower(-put);
                frontRight.setPower(-put);
                backRight.setPower(-put);
                backLeft.setPower(-put);
            }

            else if (ung>202.5 && ung<247.5) //diagonala stanga jos
            {
                frontLeft.setPower(-put);
                frontRight.setPower(0);
                backRight.setPower(-put);
                backLeft.setPower(0);
            }

            else if (ung>=247.5 && ung<=292.5) //stanga
            {
                frontLeft.setPower(-put);
                frontRight.setPower(put);
                backRight.setPower(-put);
                backLeft.setPower(put);
            }

            else if (ung>292.5 && ung<337.5) //diagonala stanga sus
            {
                frontRight.setPower(put);
                frontLeft.setPower(0);
                backLeft.setPower(put);
                backRight.setPower(0);
            }
        }
    }

    /*
     * possible control scheme:
     *      - Wheels: dpad, right stick
     *      - Servo: right bumper
     *      - LauncherWheel: right trigger
     *      - Collector: a
     *      Lift: left stick
     */

}
