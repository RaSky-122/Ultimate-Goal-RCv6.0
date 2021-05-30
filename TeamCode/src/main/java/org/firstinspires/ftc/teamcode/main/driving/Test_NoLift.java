package org.firstinspires.ftc.teamcode.main.driving;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libraries.implementations.GeneralInitImpl;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Driving", group = "main")

public class Test_NoLift extends LinearOpMode {

    public double CurrentVelocity;
    public double velocity;
    public double turretAngle;
    /*int viteza = 0;
    double power = 0.8;
    double halfPower = 0.5;
    */


    FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime runTime = new ElapsedTime();



    static final double INITIAL_ANGLE = 45;
    static final String COLLECTOR_MOTOR = "collector";
    static final String LAUNCHER_MOTOR = "launcher";
    static final String TURRET_MOTOR = "turret";
    static final String LOADING_SERVO = "loader";

    static final double TICKS_PER_DEGREE = 9.5;
    static final int MAX_DISTANCE = 124; //in inches
    static final int MAX_VELOCITY = 1800;

    static final int LOWER_LIMIT = -530;
    static final int UPPER_LIMIT = 330;




    private GeneralInitImpl init = new GeneralInitImpl();
    private Robot robot = new Robot();

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    List<DcMotor> driveMotors = new ArrayList<>();

    private DcMotorEx turretMotor;
    //Turret motor hard limits: -572 , 370
    private DcMotorEx launcherWheelMotor, collectorMotor;
    private DcMotor armWobble;
    private Servo armServo;
    private Servo loadServo;

    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard.setTelemetryTransmissionInterval(25);



        drive = new SampleMecanumDrive(hardwareMap);

        //imu = new Gyroscope().initIMU(hardwareMap);

        armWobble = hardwareMap.dcMotor.get("arm");
        armWobble.setPower(0);
        armWobble.setTargetPosition(0);
        armWobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armWobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armWobble.setDirection(DcMotorSimple.Direction.REVERSE);
        armWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretMotor = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR);
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
        launcherWheelMotor.setVelocityPIDFCoefficients(DashboardConfig.kP, DashboardConfig.kI, DashboardConfig.kD, DashboardConfig.kF);
        //pi:50, ii = 0.7, di = 100, fi = 5

        armServo = init.initServo(hardwareMap,
                "arm",
                Servo.Direction.FORWARD,
                0.5, 1);
        armServo.setPosition(1);

        loadServo = init.initServo(hardwareMap,
                LOADING_SERVO,
                Servo.Direction.FORWARD);
        loadServo.setPosition(INITIAL_ANGLE/180.0);

        collectorMotor = init.initExMotor(hardwareMap,
                COLLECTOR_MOTOR,
                DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER);

        turretMotor.setVelocityPIDFCoefficients(DashboardConfig.t_kP, DashboardConfig.t_kI, DashboardConfig.t_kD, DashboardConfig.t_kF);

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
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.setTelemetryTransmissionInterval(25);


            packet.put("Target Velocity: ", velocity);
            packet.put("Current Velocity: ", CurrentVelocity);
            packet.put("Target Angle: ", turretAngle);
            packet.put("Current Angle: ", turretMotor.getCurrentPosition()/TICKS_PER_DEGREE);

            dashboard.sendTelemetryPacket(packet);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            //smoothMovement(false); /* uses gamepad1: left stick, right stick */
            robot.localization();
            robot.turretLocalization(true); /* uses gamepad1: dpad_up */
            robot.launcherRun(true); /* uses gamepad1: right trigger */
            robot.loadRing(false); /* uses gamepad1: x */
            robot.collectorRun(false); /* uses gamepad1: a */
            robot.arm();
            telemetry.addData("","");
            telemetry.addData("","");
            telemetry.addData("Wheel velocities", drive.getWheelVelocities());

            telemetry.update();

        }
    }



    class Robot {

        private boolean turretOn = false;
        private double dist;


        private double log;
        private double heading;


        boolean launcherOn = false;

        public void localization(){
            double x1 = drive.getPoseEstimate().getX(), y1 = drive.getPoseEstimate().getY();
            double x2 = 123, y2 = -15;
            turretAngle = Math.toDegrees(Math.atan2(x1 - x2, (y1-y2))) * (-1) - 110;
            log = turretAngle;
            heading = drive.getPoseEstimate().getHeading()*180/Math.PI;

            if (heading<=180)
                turretAngle -= heading*1.16;
            else
                turretAngle += (360-heading)*1.16;
            log = turretAngle - log;

            dist = Math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
        }

        boolean armIsOn = false;
        boolean servoIsOn = false;
        int armPos = 220;
        public void arm() {
            if ((heading >= 160 && heading <= 200) && drive.getPoseEstimate().getX()<=20)
                armPos = 90;
            else
                armPos = 210;

            if (gamepad1.b && armWobble.getCurrentPosition() <= 50) {
                armWobble.setTargetPosition(armPos);
                armWobble.setPower(0.75);
                sleep(200);

            } else if (gamepad1.b && armWobble.getCurrentPosition() > 50) {
                armWobble.setTargetPosition(20);
                armWobble.setPower(0.75);
                sleep(200);
            }

            if (gamepad1.left_bumper && !servoIsOn) {
                armServo.setPosition(0);
                servoIsOn = true;
                sleep(200);
            }
            else if (gamepad1.left_bumper && servoIsOn)
            {
                armServo.setPosition(1);
                servoIsOn = false;
                sleep(200);
            }



//            if (armWobble.getCurrentPosition()>140 && gamepad1.left_bumper && armServo.getPosition()!=0){
//                armServo.setPosition(0);
//                sleep(200);
//            }
//
//            else if (armWobble.getCurrentPosition()>140 && gamepad1.left_bumper && armServo.getPosition()!=1) {
//                armServo.setPosition(1);
//                sleep(200);
//            }


            if (!armWobble.isBusy())
                armWobble.setPower(0);

            telemetry.addData("", "");
            telemetry.addData("~~~~~~~~~~~~ Arm logs ~~~~~~~~~~~~ ", "");
            telemetry.addData("Arm position: ", armWobble.getCurrentPosition());
            telemetry.addData("Arm target: ", armWobble.getTargetPosition());
            telemetry.addData("Is button pressed: ", gamepad1.b);
            telemetry.addData("Is arm on: ", armIsOn);
            telemetry.addData("Is arm busy: ", armWobble.isBusy());
            telemetry.addData("Target Pos: ", armPos);
            telemetry.addData("~~~~~~~~~~~~ Arm logs ~~~~~~~~~~~~ ", "");

        }

        public void turretLocalization(boolean getLogs){

            if(gamepad1.dpad_up){
                turretOn = !turretOn;
                sleep(200);
            }

            if(turretOn) {
                if(turretAngle*TICKS_PER_DEGREE > UPPER_LIMIT)
                    turretMotor.setTargetPosition(UPPER_LIMIT - 10);
                else if(turretAngle*TICKS_PER_DEGREE < LOWER_LIMIT)
                    turretMotor.setTargetPosition(LOWER_LIMIT + 10);
                else turretMotor.setTargetPosition((int)(turretAngle*TICKS_PER_DEGREE));
                turretMotor.setVelocity(DashboardConfig.t_velocity);
            }

            else if(!turretOn) {
                turretMotor.setVelocity(0);
            }

            if(turretMotor.getCurrentPosition() >= 400 || turretMotor.getCurrentPosition() <= -600) {
                turretMotor.setVelocity(0);
            }

            /*if(gamepad2.x){
                turretMotor.setTargetPosition(0);
                turretMotor.setPower(0.3);
                turretOn = false;
                sleep(200);
            }*/

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



        public void launcherRun(boolean getLogs){
            if (dist<=93)
                velocity = 1780-((dist-63.5)*3.50);
            else
                velocity = 1700+((dist-93)*3.50);
            telemetry.addData("Target Velocity:", velocity);

            if(gamepad1.right_trigger >= 0.7) {
                launcherOn = !launcherOn;
                sleep(200);
            }

            /*if(drive.getPoseEstimate().getX() <= 66)
                launcherOn = true;
            else launcherOn = false;

            if((turretMotor.getCurrentPosition() < UPPER_LIMIT - 15 && turretMotor.getCurrentPosition() > LOWER_LIMIT + 15) && launcherOn)
                launcherOn = true;
            else launcherOn = false;
             */

            List<Double> wheelVelocities = drive.getWheelVelocities();

            double speedAvg = (Math.abs(wheelVelocities.get(0)) + Math.abs(wheelVelocities.get(1)) + Math.abs(wheelVelocities.get(2)) + Math.abs(wheelVelocities.get(3)))/4;

            if(launcherOn){
                launcherWheelMotor.setVelocity(velocity);
            }
            else if(!launcherOn){
                launcherWheelMotor.setVelocity(0);
            }

            if(getLogs){
                CurrentVelocity = launcherWheelMotor.getVelocity();
                telemetry.addData("~~~~~~~~~~~~ Launcher ~~~~~~~~~~~~ ", "");
                telemetry.addData("Launcher velocity: ", CurrentVelocity);
                telemetry.addData("Wheel average velocity: ", speedAvg);
                telemetry.addData("~~~~~~~~~~~~ Launcher ~~~~~~~~~~~~", "");
            }
        }

        public void loadRing(boolean getLogs){

            if(gamepad1.x && loadServo.getPosition() > 40/180.0/* && launcherWheelMotor.getVelocity() >= velocity - 300*/){
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
