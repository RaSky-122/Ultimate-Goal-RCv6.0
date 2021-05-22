package org.firstinspires.ftc.teamcode.main.driving;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.hardware.Gyroscope;
import org.firstinspires.ftc.teamcode.libraries.hardware.WheelMotors;
import org.firstinspires.ftc.teamcode.libraries.implementations.GeneralInitImpl;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name = "Main Driving", group = "main")

public class MainTeleOp extends LinearOpMode {

    int viteza = 0;
    double power = 0.8;
    double halfPower = 0.5;

    ElapsedTime runTime = new ElapsedTime();

    static final double INITIAL_ANGLE = 9;

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

        liftMotor = init.initMotor(hardwareMap,
                LIFT_MOTOR,
                DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setTargetPosition(4500);
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretMotor = init.initMotor(hardwareMap,
                TURRET_MOTOR,
                DcMotor.ZeroPowerBehavior.BRAKE,
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherWheelMotor = init.initExMotor(hardwareMap,
                LAUNCHER_MOTOR,
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.REVERSE,
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
            liftToPosition(false); /* uses gamepad1: left stick */
            //liftRun(true);

            telemetry.update();

        }
    }

    private boolean launcherGoal = true;
    private int velocity = 1530;

    public void launcherRun(boolean getLogs){

        if(gamepad1.right_trigger >= 0.7 && launcherWheelMotor.getPower() == 0){
            launcherWheelMotor.setVelocity(1530); //1500 driving
            sleep(200);
        }
        else if(gamepad1.right_trigger >= 0.7 && launcherWheelMotor.getPower() != 0){
            launcherWheelMotor.setVelocity(0);
            sleep(200);
        }

        if(gamepad1.b && launcherGoal){
            launcherGoal = false;
            velocity = 1370;
        }
        else if(gamepad1.b && !launcherGoal){
            launcherGoal = true;
            velocity = 1500;
        }

        if(getLogs){
            telemetry.addData("Launcher velocity ", launcherWheelMotor.getVelocity());
        }
    }

    public void loadRing(boolean getLogs){

        if(gamepad1.x && loadServo.getPosition() < 25/180.0){
            runTime.reset();
            loadServo.setPosition(37/180.0);
            sleep(200);
        }
        else if (loadServo.getPosition() > 15/180.0 && runTime.milliseconds() > 350){
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

    public boolean top = false;

    public void liftToPosition(boolean getLogs){ /* gamepad left stick y */

        if(top){
            collectorMotor.setPower(0);
            if(liftMotor.getCurrentPosition() > 3000)
                launcherWheelMotor.setVelocity(velocity);
            loadServo.setPosition(INITIAL_ANGLE/180.0);
        }
        else if(!top){
            launcherWheelMotor.setPower(0);
        }

        if(-gamepad1.left_stick_y > 0.7 && !top){
            top = true;
            liftMotor.setTargetPosition(4345); //lift top is 4571
            liftMotor.setPower(1);
            sleep(200);
        }
        else if(-gamepad1.left_stick_y < -0.7 && top){
            top = false;
            liftMotor.setTargetPosition(25);
            liftMotor.setPower(1);
            sleep(200);
        }

        if(!liftMotor.isBusy())
            liftMotor.setPower(0);

        if(getLogs) {
            telemetry.addData("Lift power ", liftMotor.getPower());
            telemetry.addData("Lift encoder ", liftMotor.getCurrentPosition());
        }
    }

    public void liftRun(boolean getLogs){

        if(gamepad1.y && liftMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else if(gamepad1.y)
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(-gamepad1.left_stick_y > 0.3){
            if(liftMotor.getCurrentPosition() > 3000)
                launcherWheelMotor.setVelocity(1500);
            collectorMotor.setPower(0);

            liftMotor.setTargetPosition(4500);
            liftMotor.setPower(1);

            loadServo.setPosition(INITIAL_ANGLE/180.0);

            top = true;
            sleep(200);
        }
        else if(-gamepad1.left_stick_y < -0.3){
            launcherWheelMotor.setPower(0);

            liftMotor.setTargetPosition(25);
            liftMotor.setPower(-1);

            top = false;
            sleep(200);
        }
        else liftMotor.setPower(0);

        if(getLogs){
            telemetry.addData("Lift power ", liftMotor.getPower());
            telemetry.addData("Lift encoder ", liftMotor.getCurrentPosition());
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

    /*
     * possible control scheme:
     *      - Wheels: dpad, right stick
     *      - Servo: right bumper
     *      - LauncherWheel: right trigger
     *      - Collector: a
     *      Lift: left stick
     */
}
