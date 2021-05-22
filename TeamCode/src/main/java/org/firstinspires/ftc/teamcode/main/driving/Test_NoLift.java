package org.firstinspires.ftc.teamcode.main.driving;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libraries.hardware.Gyroscope;
import org.firstinspires.ftc.teamcode.libraries.hardware.WheelMotors;
import org.firstinspires.ftc.teamcode.libraries.implementations.GeneralInitImpl;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Driving", group = "main")

public class Test_NoLift extends LinearOpMode {

    int viteza = 0;
    double power = 0.8;
    double halfPower = 0.5;

    ElapsedTime runTime = new ElapsedTime();

    static final double INITIAL_ANGLE = 45;
    static double dist;
    static final String COLLECTOR_MOTOR = "collector";
    static final String LAUNCHER_MOTOR = "launcher";
    static final String TURRET_MOTOR = "turret";
    static final String LIFT_MOTOR = "lift";
    static final String LOADING_SERVO = "loader";

    static final double TICKS_PER_DEGREE = 9.5;
    static final int MAX_DISTANCE = 124; //in inches
    static final int MAX_VELOCITY = 1800;

    private GeneralInitImpl init = new GeneralInitImpl();

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    List<DcMotor> driveMotors = new ArrayList<>();

    private DcMotor turretMotor, collectorMotor;
    private double turretAngle;
    //Turret motor hard limits: -572 , 370
    private DcMotorEx launcherWheelMotor;

    private Servo loadServo;

    //private BNO055IMU imu;

    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //imu = new Gyroscope().initIMU(hardwareMap);

        turretMotor = hardwareMap.dcMotor.get(TURRET_MOTOR);
        turretMotor.setPower(0);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        /*driveMotors = new WheelMotors().initWheels(hardwareMap,
                DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft = driveMotors.get(0); frontRight = driveMotors.get(1);
        backLeft = driveMotors.get(2); backRight = driveMotors.get(3);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         */

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Waiting for start", "");
            telemetry.update();
        }

        while(opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            //smoothMovement(false); /* uses gamepad1: left stick, right stick */
            turretLocalization(true); /* uses gamepad1: dpad_up */
            launcherRun(true); /* uses gamepad1: right trigger */
            loadRing(false); /* uses gamepad1: x */
            collectorRun(false); /* uses gamepad1: a */

            telemetry.update();

        }
    }

    private boolean turretOn = false;

    public void turretLocalization(boolean getLogs){
        double x1 = drive.getPoseEstimate().getX(), y1 = drive.getPoseEstimate().getY();
        double x2 = 123, y2 = -15;
        turretAngle = Math.toDegrees(Math.atan2(x1 - x2, (y1-y2))) * (-1) - 110;
        double log = turretAngle;
        double heading = drive.getPoseEstimate().getHeading()*180/Math.PI;

        if (heading<=180)
            turretAngle -= heading*1.2;
        else
            turretAngle += (360-heading)*1.2;
        log = turretAngle - log;

        dist = Math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));

        if(gamepad1.dpad_up){
            turretOn = !turretOn;
            sleep(200);
        }

        if(turretOn) {
            if(turretAngle*TICKS_PER_DEGREE > 350)
                turretMotor.setTargetPosition(340);
            else if(turretAngle*TICKS_PER_DEGREE < -550)
                turretMotor.setTargetPosition(-540);
            else turretMotor.setTargetPosition((int)(turretAngle*TICKS_PER_DEGREE));
            turretMotor.setPower(0.5);
        }
        /*else if(!gamepad1.dpad_up && turretMotor.getCurrentPosition() >= 360) {
            turretMotor.setTargetPosition(360);
            turretMotor.setPower(0.3);
        }
        else if(!gamepad1.dpad_up && turretMotor.getCurrentPosition() <= -560){
            turretMotor.setTargetPosition(-560);
            turretMotor.setPower(0.3);
        }*/
        else if(!turretOn) {
            turretMotor.setPower(0);
        }

        if(turretMotor.getCurrentPosition() >= 400 || turretMotor.getCurrentPosition() <= -600) {
            turretMotor.setPower(0);
        }

        if(gamepad2.x){
            turretMotor.setTargetPosition(0);
            turretMotor.setPower(0.3);
            turretOn = false;
            sleep(200);
        }

        if(!turretMotor.isBusy())
            turretMotor.setPower(0);

        if(getLogs) {
            telemetry.addData("~~~~~~~~~~~~ Turret localization ~~~~~~~~~~~~ ", "");
            telemetry.addData("Target position: ", turretMotor.getTargetPosition());
            telemetry.addData("Current position: ", turretMotor.getCurrentPosition());
            telemetry.addData("Angle to net: ", turretAngle);
            telemetry.addData("Turret motor power: ", turretMotor.getPower());
            telemetry.addData("Target with heading: ", log);
            telemetry.addData("Distance to net: ", dist);
            telemetry.addData("Pose X: ", drive.getPoseEstimate().getX());
            telemetry.addData("Pose Y: ", drive.getPoseEstimate().getY());
            telemetry.addData("Heading 2: ", heading);
            telemetry.addData("~~~~~~~~~~~~ Turret localization ~~~~~~~~~~~~ ", "end ");
        }
    }

    boolean isRunning = false;

    public void launcherRun(boolean getLogs){

        double velocity;
        if (dist<=93)
            velocity = 1780-(dist-63.5)*3.75;
        else
            velocity = 1700;
        telemetry.addData("Target Velocity:", velocity);

        if(gamepad1.right_trigger >= 0.7) {
            isRunning = !isRunning;
            sleep(200);
        }

        if(isRunning){
            launcherWheelMotor.setVelocity(velocity);
        }
        else if(!isRunning){
            launcherWheelMotor.setVelocity(0);
        }

        if(getLogs){
            telemetry.addData("Launcher velocity ", launcherWheelMotor.getVelocity());
        }
    }

    public void loadRing(boolean getLogs){

        if(gamepad1.x && loadServo.getPosition() > 40/180.0){
            runTime.reset();
            loadServo.setPosition(23/180.0);
            sleep(350);
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

    /*
    public void smoothMovement(boolean getLogs) throws InterruptedException {

        if(getLogs){
            telemetry.addData("~~~~~~~~~~~~~~~~~~~~", "");
            telemetry.addData("Front left ", frontLeft.getCurrentPosition());
            telemetry.addData("Front right ", frontRight.getCurrentPosition());
            telemetry.addData("Back left ", backLeft.getCurrentPosition());
            telemetry.addData("Back right ", backRight.getCurrentPosition());
            telemetry.addData("~~~~~~~~~~~~~~~~~~~~", "");
        }

        if(gamepad1.right_bumper && viteza == 0){
            viteza = 1;
            Thread.sleep(200);
        }
        else
        if(gamepad1.right_bumper && viteza == 1) {
            viteza = 0;
            Thread.sleep(200);
        }

        if(viteza == 0){
            if(gamepad1.dpad_up){
                frontLeft.setPower(power+gamepad1.right_stick_x/2);
                frontRight.setPower(power-gamepad1.right_stick_x/2);
                backRight.setPower(power-gamepad1.right_stick_x/2);
                backLeft.setPower(power+gamepad1.right_stick_x/2);
            }else
            if(gamepad1.dpad_down){
                frontLeft.setPower(-power+gamepad1.right_stick_x/2);
                frontRight.setPower(-power-gamepad1.right_stick_x/2);
                backRight.setPower(-power-gamepad1.right_stick_x/2);
                backLeft.setPower(-power+gamepad1.right_stick_x/2);
            }else
            if(gamepad1.dpad_right){
                frontLeft.setPower(power+gamepad1.right_stick_x/2);
                frontRight.setPower(-power-gamepad1.right_stick_x/2);
                backRight.setPower(power-gamepad1.right_stick_x/2);
                backLeft.setPower(-power+gamepad1.right_stick_x/2);
            }else
            if(gamepad1.dpad_left){
                frontLeft.setPower(-power+gamepad1.right_stick_x/2);
                frontRight.setPower(power-gamepad1.right_stick_x/2);
                backRight.setPower(-power-gamepad1.right_stick_x/2);
                backLeft.setPower(power+gamepad1.right_stick_x/2);
            }else{
                frontLeft.setPower(+gamepad1.right_stick_x+frontLeft.getPower()/4);
                frontRight.setPower(-gamepad1.right_stick_x+frontRight.getPower()/4);
                backRight.setPower(-gamepad1.right_stick_x+backRight.getPower()/4);
                backLeft.setPower(+gamepad1.right_stick_x+backLeft.getPower()/4);
            }
        }
        else if(viteza == 1){
            if(gamepad1.dpad_up){
                frontLeft.setPower(halfPower+gamepad1.right_stick_x/2);
                frontRight.setPower(halfPower-gamepad1.right_stick_x/2);
                backRight.setPower(halfPower-gamepad1.right_stick_x/2);
                backLeft.setPower(halfPower+gamepad1.right_stick_x/2);
            }else
            if(gamepad1.dpad_down){
                frontLeft.setPower(-halfPower+gamepad1.right_stick_x/2);
                frontRight.setPower(-halfPower-gamepad1.right_stick_x/2);
                backRight.setPower(-halfPower-gamepad1.right_stick_x/2);
                backLeft.setPower(-halfPower+gamepad1.right_stick_x/2);
            }else
            if(gamepad1.dpad_right){
                frontLeft.setPower(halfPower+gamepad1.right_stick_x/2);
                frontRight.setPower(-halfPower-gamepad1.right_stick_x/2);
                backRight.setPower(halfPower-gamepad1.right_stick_x/2);
                backLeft.setPower(-halfPower+gamepad1.right_stick_x/2);
            }else
            if(gamepad1.dpad_left){
                frontLeft.setPower(-halfPower+gamepad1.right_stick_x/2);
                frontRight.setPower(halfPower-gamepad1.right_stick_x/2);
                backRight.setPower(-halfPower-gamepad1.right_stick_x/2);
                backLeft.setPower(halfPower+gamepad1.right_stick_x/2);
            }else{
                frontLeft.setPower(+gamepad1.right_stick_x/2 + frontLeft.getPower()/2);
                frontRight.setPower(-gamepad1.right_stick_x/2 + frontRight.getPower()/2);
                backRight.setPower(-gamepad1.right_stick_x/2 + backRight.getPower()/2);
                backLeft.setPower(+gamepad1.right_stick_x/2 + backLeft.getPower()/2);
            }
        }

    }
     */

    /*
     * control scheme:
     *      - Wheels: left stick, right stick
     *      - Servo: x
     *      - LauncherWheel: right trigger
     *      - Collector: a
     *      - Turret movement: dpad_up
     */

}
