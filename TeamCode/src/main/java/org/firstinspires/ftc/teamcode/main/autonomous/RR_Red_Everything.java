package org.firstinspires.ftc.teamcode.main.autonomous;

import android.graphics.Camera;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libraries.implementations.GeneralInitImpl;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "RR Auton", group = "main")
public class RR_Red_Everything extends LinearOpMode {

    //~~~~~~~~~~~~~~~~~~~ W E B C A M ~~~~~~~~~~~~~~~~~~~~~

    private static final String VUFORIA_KEY =
            "Ab7Zg4v/////AAABmZKeB/V4p0DYmJFYkJRpaeou+GcJbvvo75+A7Spuy3TFuR7rC0nOwuzdeXJ7XvtxlPAkZYwkdO/EW6CU6mfG8AEqTsAxy//INcvlqdvdnkZeWG3qKsllZorOFWuKfCcrIM3TyA7iOsG2gl0jPG7+PF2S6kpYhHhJ4h1B+9l1B/dx/pQi96ktSn5L8Df3/sAn9WIPlcmRtcByc7N1/cA7liBfwaeiY+HVJZbtuJrxUg+LRJumxU6xjh0Cgq+OcyvXeMXA15i7i/5js/jFa+AqV8jIDNGZpdOyVlgrpSV+/bZsLcnFFY9GSIgYXHbLpFtkv6tQpUd/4kamH3qyxvFRhh5w9uXPVa1Q0xMiLQhUxiuD";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private Camera camera = new Camera();

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    //~~~~~~~~~~~~~~~~~~~ W E B C A M ~~~~~~~~~~~~~~~~~~~~~

    ElapsedTime runTime = new ElapsedTime();

    static final double TICKS_PER_DEGREE = 9.5;
    static final double INITIAL_ANGLE = 45;

    private DcMotor turretMotor;
    private double turretAngle;
    private DcMotorEx launcherWheelMotor;
    private Servo loadServo;

    static double dist;

    static final String COLLECTOR_MOTOR = "collector";
    static final String LAUNCHER_MOTOR = "launcher";
    static final String TURRET_MOTOR = "turret";
    static final String LIFT_MOTOR = "lift";
    static final String LOADING_SERVO = "loader";

    private SampleMecanumDrive drive;

    private GeneralInitImpl init = new GeneralInitImpl();

    @Override
    public void runOpMode() throws InterruptedException {

        //~~~~~~~~~~~~~~~~~~~ W E B C A M ~~~~~~~~~~~~~~~~~~~~~

        new Init().vuforia();
        new Init().tfod();

        if(tfod != null)
            tfod.activate();

        //~~~~~~~~~~~~~~~~~~~ W E B C A M ~~~~~~~~~~~~~~~~~~~~~

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

        drive = new SampleMecanumDrive(hardwareMap);
        ArrayList<Trajectory> traj = new ArrayList<>();
        Trajectory trajFinal;

        DriveConstants driveConstants = new DriveConstants();

        /*                E X A M P L E   T R A J E C T O R Y   W I T H   C H A N G E D   V E L O C I T Y

        drive.trajectoryBuilder(new Pose2d())
                .forward(30, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        */

        /*traj.add(drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(27, 10), 0)
                .splineToConstantHeading(new Vector2d(58, 0), 0)
                .build());
        */

        traj.add()


        waitForStart();

        int nrDiscs = discArrangement();

        if (isStopRequested()) return;

        drive.followTrajectory(traj.get(0));
        turretLocalization(true);
        launcherRun(1755, true);

        loadRing(false);
        loadRing(false);
        loadRing(false);

        sleep(2000);
    }

    class Init {
        private void vuforia() {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }

        private void tfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = (float)0.5;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        }
    }

    private int discArrangement(){
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if(updatedRecognitions != null){
            for(Recognition recognition : updatedRecognitions){
                if(recognition.getLabel().equals(LABEL_SECOND_ELEMENT))
                    return 1;
                else if(recognition.getLabel().equals(LABEL_FIRST_ELEMENT))
                    return 4;
            }
        }
        return 0;
    }

    public void turretLocalization(boolean getLogs){

        double x1 = drive.getPoseEstimate().getX();
        double y1 = drive.getPoseEstimate().getY();
        double x2 = 123, y2 = -15;

        turretAngle = Math.toDegrees(Math.atan2(/*drive.getPoseEstimate().getX()-123*/
                x1 - x2, /*drive.getPoseEstimate().getY()+15))*(-1) - 105*/
                (y1-y2))) * (-1) - 110;

        double log = turretAngle;
        if (drive.getPoseEstimate().getHeading()*60<=185)
            turretAngle -= drive.getPoseEstimate().getHeading()*60;
        else
            turretAngle += (377-drive.getPoseEstimate().getHeading()*60);
        log = turretAngle - log;

        dist = Math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));

        if(turretAngle*TICKS_PER_DEGREE > 350)
            turretMotor.setTargetPosition(340);
        else if(turretAngle*TICKS_PER_DEGREE < -550)
            turretMotor.setTargetPosition(-540);
        else turretMotor.setTargetPosition((int)(turretAngle*TICKS_PER_DEGREE));

        turretMotor.setPower(0.5);

        while (Math.abs(turretMotor.getCurrentPosition()) <= Math.abs(turretMotor.getTargetPosition())) {

            telemetry.addData("~~~~~~~~~~~~ Turret localization ~~~~~~~~~~~~ ", "");
            telemetry.addData("Target position: ", turretMotor.getTargetPosition());
            telemetry.addData("Current position: ", turretMotor.getCurrentPosition());
            telemetry.addData("Mode:", turretMotor.getMode());
            telemetry.addData("Angle to net: ", turretAngle);
            telemetry.addData("Turret motor power: ", turretMotor.getPower());
            telemetry.addData("Target with heading: ", log);
            telemetry.addData("Distance to net: ", dist);
            telemetry.addData("Pose X: ", drive.getPoseEstimate().getX());
            telemetry.addData("Pose Y: ", drive.getPoseEstimate().getY());
            telemetry.addData("Heading: ", drive.getPoseEstimate().getHeading() * 60);
            telemetry.addData("~~~~~~~~~~~~ Turret localization ~~~~~~~~~~~~ ", "end ");
            telemetry.update();
        }

        turretMotor.setPower(0);
    }

    public void launcherRun(double velocity, boolean getLogs){

        launcherWheelMotor.setVelocity(velocity); // 1755-ish? for just before the discs
        sleep(1500);

        if(getLogs){
            telemetry.addData("Launcher velocity ", launcherWheelMotor.getVelocity());
        }
    }

    public void loadRing(boolean getLogs){

        runTime.reset();
        loadServo.setPosition(23 / 180.0);
        sleep(500);

        loadServo.setPosition(INITIAL_ANGLE/180.0);
        sleep(450);

        if(getLogs){
            telemetry.addData("Load servo position ", loadServo.getPosition());
        }
    }


}