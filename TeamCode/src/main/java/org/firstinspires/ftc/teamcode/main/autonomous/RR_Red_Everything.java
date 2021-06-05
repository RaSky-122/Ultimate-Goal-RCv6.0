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

import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.math.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libraries.implementations.GeneralInitImpl;
import org.firstinspires.ftc.teamcode.main.driving.DashboardConfig;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

@Autonomous(name = "RR Auton", group = "main")
public class RR_Red_Everything extends LinearOpMode {

    enum Targets{
        GOAL,
        PSHOT1,
        PSHOT2,
        PSHOT3
    }

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

    private DcMotorEx turretMotor;
    private double turretAngle;
    private DcMotorEx launcherWheelMotor, collectorMotor;
    private Servo loadServo;

    static double dist;

    private DcMotor armWobble;
    private Servo armServo;

    static final String COLLECTOR_MOTOR = "collector";
    static final String LAUNCHER_MOTOR = "launcher";
    static final String TURRET_MOTOR = "turret";
    static final String LOADING_SERVO = "loader";

    static final int LOWER_LIMIT = -530;
    static final int UPPER_LIMIT = 330;
    int target = 0;
    private double log;
    private double heading;

    private SampleMecanumDrive drive;

    private GeneralInitImpl init = new GeneralInitImpl();

    @Override
    public void runOpMode() throws InterruptedException {


        turretMotor = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR);
        turretMotor.setPower(0);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setVelocityPIDFCoefficients(DashboardConfig.t_kP, DashboardConfig.t_kI, DashboardConfig.t_kD, DashboardConfig.t_kF);


        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        launcherWheelMotor = init.initExMotor(hardwareMap,
                LAUNCHER_MOTOR,
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER);
        launcherWheelMotor.setVelocityPIDFCoefficients(400, 0.8,35, 3);

        collectorMotor = init.initExMotor(hardwareMap,
                COLLECTOR_MOTOR,
                DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER);

        loadServo = init.initServo(hardwareMap,
                LOADING_SERVO,
                Servo.Direction.FORWARD);
        loadServo.setPosition(INITIAL_ANGLE/180.0);

        armWobble = hardwareMap.dcMotor.get("arm");
        armWobble.setPower(0);
        armWobble.setTargetPosition(0);
        armWobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armWobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armWobble.setDirection(DcMotorSimple.Direction.REVERSE);
        armWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armServo = init.initServo(hardwareMap,
                "arm",
                Servo.Direction.FORWARD,
                0.5, 1);
        armServo.setPosition(0);

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

        traj.add(drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(27, 10), 0)
                .splineToConstantHeading(new Vector2d(58, 0), 0)
                .build());


        //~~~~~~~~~~~~~~~~~~~ W E B C A M ~~~~~~~~~~~~~~~~~~~~~

        new Init().vuforia();
        new Init().tfod();

        if(tfod != null)
            tfod.activate();

        telemetry.addData("Waiting for start", "");
        telemetry.update();
        //~~~~~~~~~~~~~~~~~~~ W E B C A M ~~~~~~~~~~~~~~~~~~~~~

        waitForStart();

        int nrDiscs = discArrangement();

        if (isStopRequested()) return;

        // powershot: 1 = 46, 2 = 65, 3 = 84 //


//        launcherRun(1730, false);


//        while (!gamepad1.b)
//        sleep(10);
//
//        turretLocalization(++target,false);
//        while (!gamepad1.b)
//            loadRing(false);
//
//        turretLocalization(++target, false);
//        while (!gamepad1.b)
//            loadRing(false);
//
//        turretLocalization(++target, false);
//        while (!gamepad1.b)
//            loadRing(false);

        if (nrDiscs == 1)
            launcherRun(1745, true);
        else if (nrDiscs == 0)
            launcherRun(1725, true);

        if(nrDiscs == 1)
            drive.followTrajectory(traj.get(0));
        else traj.clear();


        if (nrDiscs == 1) {
            turretLocalization(true);
            telemetry.addData("Putere", launcherWheelMotor.getVelocity());
            telemetry.update();
            sleep(400);
            loadRing(false);

            telemetry.addData("Putere", launcherWheelMotor.getVelocity());
            telemetry.update();
            sleep(300);
            loadRing(false);
            telemetry.update();

            telemetry.addData("Putere", launcherWheelMotor.getVelocity());
            telemetry.update();
            sleep(300);
            loadRing(false);
            telemetry.update();

            launcherRun(0, false);
            telemetry.addData("Discuri: ", nrDiscs);
            telemetry.update();
        }
        if (nrDiscs == 1)
        {
            traj.add(drive.trajectoryBuilder(traj.get(0).end())
                    .lineTo(new Vector2d(85, -2))
                    .build());

            drive.followTrajectory(traj.get(1));
            armWobble.setTargetPosition(220);
            armWobble.setPower(0.75);
            sleep(100);
            while (armWobble.getCurrentPosition() <= 220) ;
            armWobble.setPower(0);
            armServo.setPosition(1);

            collectorMotor.setPower(1);

            traj.add(drive.trajectoryBuilder(traj.get(1).end(), true)
                    .splineToSplineHeading(new Pose2d(52, -10, Math.toRadians(-168)), Math.toRadians(180)
                    ,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, 5.0, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .splineToSplineHeading(new Pose2d(27, -11, Math.toRadians(-118)), Math.toRadians(180))
                    .build());
            drive.followTrajectory(traj.get(2));

            drive.followTrajectory(drive.trajectoryBuilder(traj.get(2).end())
                    .forward(3.75)
                    .build());

            armServo.setPosition(0);
            sleep(75);
            armWobble.setTargetPosition(50);
            armWobble.setPower(0.75);
            sleep(175);
            while (armWobble.getCurrentPosition() >= 50);
            armWobble.setPower(0);

            collectorMotor.setPower(0);

            traj.add(drive.trajectoryBuilder(traj.get(2).end())
                    .lineToLinearHeading(new Pose2d(58, -2, Math.toRadians(0)))
                    .build());
            launcherRun(DashboardConfig.l_velocity, false);

            drive.followTrajectory(traj.get(3));

            turretLocalization(false);
            loadRing(false);

            traj.add(drive.trajectoryBuilder(traj.get(3).end())
                    .forward(80 - traj.get(3).end().getX()-4.5)
                    .build());

            drive.followTrajectory(traj.get(4));

            armWobble.setTargetPosition(220);
            armWobble.setPower(0.75);
            sleep(100);

            while (armWobble.getCurrentPosition() <= 220) ;
            armWobble.setPower(0);

            armServo.setPosition(1);

            armWobble.setTargetPosition(20);
            armWobble.setPower(0.75);
            sleep(100);

            while (armWobble.getCurrentPosition() >= 20) ;
            armWobble.setPower(0);
        }
        else if(nrDiscs == 0){
            traj.add(drive.trajectoryBuilder(new Pose2d())
                    .lineToConstantHeading(new Vector2d(58, -30))
                    .build());
            drive.followTrajectory(traj.get(0));

            turretLocalization(true);
            telemetry.addData("Putere", launcherWheelMotor.getVelocity());
            telemetry.update();
            sleep(400);
            loadRing(false);

            telemetry.addData("Putere", launcherWheelMotor.getVelocity());
            telemetry.update();
            sleep(300);
            loadRing(false);
            telemetry.update();

            telemetry.addData("Putere", launcherWheelMotor.getVelocity());
            telemetry.update();
            sleep(300);
            loadRing(false);
            telemetry.update();

            launcherRun(0, false);
            telemetry.addData("Discuri: ", nrDiscs);
            telemetry.update();

            armWobble.setTargetPosition(220);
            armWobble.setPower(0.75);
            sleep(100);
            while (armWobble.getCurrentPosition() <= 220) ;
            armWobble.setPower(0);
            armServo.setPosition(1);

            armWobble.setTargetPosition(20);
            armWobble.setPower(0.75);
            sleep(100);
            while (armWobble.getCurrentPosition() >= 20) ;
            armWobble.setPower(0);

            traj.add(drive.trajectoryBuilder(traj.get(0).end())
                    .lineToLinearHeading(new Pose2d(27, -10, Math.toRadians(-118)))
                    .build());
            drive.followTrajectory(traj.get(1));

            armWobble.setTargetPosition(220);
            armWobble.setPower(0.75);
            sleep(100);
            while (armWobble.getCurrentPosition() <= 220) ;
            armWobble.setPower(0);
            armServo.setPosition(1);

            traj.add(drive.trajectoryBuilder(traj.get(1).end())
                    .forward(4.5)
                    .build());
            
            drive.followTrajectory(traj.get(2));

            armServo.setPosition(0);
            sleep(100);
            armWobble.setTargetPosition(20);
            armWobble.setPower(0.75);
            sleep(100);
            while (armWobble.getCurrentPosition() >= 20) ;
            armWobble.setPower(0);

            traj.add(drive.trajectoryBuilder(traj.get(2).end())
                    .lineToLinearHeading(new Pose2d(50, -30, 0))
                    .build());
            drive.followTrajectory(traj.get(3));

            armWobble.setTargetPosition(220);
            armWobble.setPower(0.75);
            sleep(100);
            while (armWobble.getCurrentPosition() <= 220) ;
            armWobble.setPower(0);
            armServo.setPosition(1);

            sleep(100);
            armWobble.setTargetPosition(20);
            armWobble.setPower(0.75);
            sleep(100);
            while (armWobble.getCurrentPosition() >= 20) ;
            armWobble.setPower(0);

            traj.add(drive.trajectoryBuilder(traj.get(3).end())
                    .lineToLinearHeading(new Pose2d(70, -15, Math.toRadians(45)))
                    .build());
            drive.followTrajectory(traj.get(4));
        }
        else if (nrDiscs==4)
        {
            traj.add(drive.trajectoryBuilder(new Pose2d())
                    .lineTo(new Vector2d(25, -11.5))
                    .build());
            launcherRun(1675, false);
            sleep(100);
            drive.followTrajectory(traj.get(0));
            turretLocalization(false);
            sleep(400);
            loadRing(false);
            sleep(400);
            loadRing(false);
            sleep(400);
            loadRing(false);

            traj.add(drive.trajectoryBuilder(traj.get(0).end())
                    .forward(9, SampleMecanumDrive.getVelocityConstraint(3.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build());
            collectorMotor.setPower(1);
            launcherRun(1620, false);
            drive.followTrajectory(traj.get(1));
            turretLocalization(false);
            sleep(1500);
            loadRing(false);

            traj.add(drive.trajectoryBuilder(traj.get(1).end())
                    .forward(20, SampleMecanumDrive.getVelocityConstraint(5.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build());

            launcherRun(1665, false);
            drive.followTrajectory(traj.get(2));
            collectorMotor.setPower(0);
            sleep(150);

            turretLocalization(false);
            sleep(400);
            loadRing(false);
            sleep(400);
            loadRing(false);
            sleep(400);
            loadRing(false);

            launcherRun(0, false);
        }

        try {
            FileOutputStream outputStream = new FileOutputStream("org/firstinspires/ftc/teamcode/main/FieldPos.txt");
            DataOutputStream writer = new DataOutputStream(outputStream);
            writer.writeDouble(drive.getPoseEstimate().getX());
            writer.writeDouble(drive.getPoseEstimate().getY());
            writer.writeDouble(drive.getPoseEstimate().getHeading());
        } catch(IOException e){
            System.out.println("Error writing to file!");
        }
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
            tfodParameters.minResultConfidence = (float)0.60;
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

//    public void turretLocalization(int target, boolean getLogs){
//
//        telemetry.addData("Target label: ", target);
//        if(target == 1){
//            y2 += 18.110236220;
//        }
//        if(target == 2){
//            y2 += 7.48031496;
//        }
//        if(target == 3){
//            y2 += 7.48031496;
//        }
//
//        double x1 = drive.getPoseEstimate().getX();
//        double y1 = drive.getPoseEstimate().getY();
//
//        turretAngle = Math.toDegrees(Math.atan2(x1 - x2, (y1-y2) * (-1) - 110));
//
//        double log = turretAngle;
//        if (drive.getPoseEstimate().getHeading()*60<=185)
//            turretAngle -= drive.getPoseEstimate().getHeading()*60;
//        else
//            turretAngle += (377-drive.getPoseEstimate().getHeading()*60);
//        log = turretAngle - log;
//
//        dist = Math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
//
//        if(turretAngle*TICKS_PER_DEGREE > 350)
//            turretMotor.setTargetPosition(340);
//        else if(turretAngle*TICKS_PER_DEGREE < -550)
//            turretMotor.setTargetPosition(-540);
//        else turretMotor.setTargetPosition((int)(turretAngle*TICKS_PER_DEGREE));
//
//        turretMotor.setPower(0.5);
//
//        if (!turretMotor.isBusy())
//            turretMotor.setPower(0);
//
//
//
//
//            telemetry.addData("~~~~~~~~~~~~ Turret localization ~~~~~~~~~~~~ ", "");
//            telemetry.addData("Angle: ", turretAngle);
//            telemetry.addData("Target position: ", turretMotor.getTargetPosition());
//            telemetry.addData("Current position: ", turretMotor.getCurrentPosition());
//            telemetry.addData("Target y position: ", y2);
//            telemetry.addData("~~~~~~~~~~~~ Turret localization ~~~~~~~~~~~~ ", "end ");
//            telemetry.update();
//    }

    public void localization(){
        double x1 = drive.getPoseEstimate().getX(), y1 = drive.getPoseEstimate().getY();
        double x2 = 123, y2 = -15;
        switch (target)
        {
            case 1:
                y2 = DashboardConfig.powershot1;
                break;
            case 2:
                y2 = DashboardConfig.powershot2;
                break;
            case 3:
                y2 = DashboardConfig.powershot3;
                break;
        }

        turretAngle = Math.toDegrees(Math.atan2(x1 - x2, (y1-y2))) * (-1) - 113;
        log = turretAngle;
        heading = drive.getPoseEstimate().getHeading()*180/Math.PI;

        if (heading<=180)
            turretAngle -= heading*1.16;
        else
            turretAngle += (360-heading)*1.16;
        log = turretAngle - log;

        dist = Math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }

    public void turretLocalization(boolean getLogs){

            localization();

            if(turretAngle*TICKS_PER_DEGREE > UPPER_LIMIT)
                turretMotor.setTargetPosition(UPPER_LIMIT - 10);
            else if(turretAngle*TICKS_PER_DEGREE < LOWER_LIMIT)
                turretMotor.setTargetPosition(LOWER_LIMIT + 10);
            else turretMotor.setTargetPosition((int)(turretAngle*TICKS_PER_DEGREE));
            turretMotor.setVelocity(DashboardConfig.t_velocity);

//             while (true)
//                 if (!turretMotor.isBusy()) {
//                 turretMotor.setPower(0);
//                 break;
//                 }

        if(turretMotor.getCurrentPosition() >= 400 || turretMotor.getCurrentPosition() <= -600) {
            turretMotor.setVelocity(0);
        }

        //if(!turretMotor.isBusy())
            //turretMotor.setPower(0);

        if(getLogs) {
            telemetry.addData("~~~~~~~~~~~~ Turret localization ~~~~~~~~~~~~ ", "");
            telemetry.addData("Target position: ", turretMotor.getTargetPosition());
            telemetry.addData("Current position: ", turretMotor.getCurrentPosition());
            telemetry.addData("Angle to net: ", turretAngle);
            telemetry.addData("Turret motor power: ", turretMotor.getPower());
//            telemetry.addData("Target with heading: ", log);
            telemetry.addData("Distance to net: ", dist);
            telemetry.addData("Pose X: ", drive.getPoseEstimate().getX());
            telemetry.addData("Pose Y: ", drive.getPoseEstimate().getY());
//            telemetry.addData("Heading 2: ", heading);
            telemetry.addData("~~~~~~~~~~~~ Turret localization ~~~~~~~~~~~~ ", "end ");
        }
    }


    public void launcherRun(double velocity, boolean getLogs){

        launcherWheelMotor.setVelocity(velocity); // 1755-ish? for just before the discs
//        sleep(1500);

        if(getLogs){
            telemetry.addData("Launcher velocity ", launcherWheelMotor.getVelocity());
        }
    }

    public void loadRing(boolean getLogs){

        runTime.reset();
        loadServo.setPosition(23 / 180.0);
        sleep(500);

        loadServo.setPosition(INITIAL_ANGLE/180.0);

        if(getLogs){
            telemetry.addData("Load servo position ", loadServo.getPosition());
        }
    }


}