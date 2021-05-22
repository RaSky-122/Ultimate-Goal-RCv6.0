package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "Launcher Test", group = "test")
public class LauncherTest extends LinearOpMode{

    private DcMotorEx LauncherMotor, TurretMotor;

    @Override
    public void runOpMode() throws InterruptedException{

        TurretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        TurretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        TurretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LauncherMotor = hardwareMap.get(DcMotorEx.class, "launcher");

        LauncherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Waiting for start", "");
            telemetry.update();
        }

        boolean isPressed = false;
        double motorPower = 0;

        while(opModeIsActive()){

            if(gamepad1.x && !isPressed){
                isPressed = true;
                if(TurretMotor.getCurrentPosition() < 50)
                    while(TurretMotor.getCurrentPosition() <= 200){
                        TurretMotor.setPower(0.5);
                    }
                else if(TurretMotor.getCurrentPosition() > 150)
                    while(TurretMotor.getCurrentPosition() >= 0){
                        TurretMotor.setPower(-0.5);
                    }
                else TurretMotor.setPower(0);
            }
            else TurretMotor.setPower(0);

            if(gamepad1.y && !isPressed) {

                isPressed = true;
                motorPower += 0.05;
            }
            else if (gamepad1.a && !isPressed){

                isPressed = true;
                motorPower -= 0.05;
            }
            else if(!gamepad1.a && !gamepad1.y && !gamepad1.x)
                isPressed = false;

            if(gamepad1.dpad_up) {

                LauncherMotor.setPower(motorPower);
            }
            else if(gamepad1.dpad_down) {

                LauncherMotor.setPower(-motorPower);
            }
            else {

                LauncherMotor.setPower(0);
            }

            telemetry.addData("Turret rotate: ", "X button");
            telemetry.addData("Launcher set speed: ", "A -> Lower ... Y -> Higher");
            telemetry.addData("Launcher on: ", "DPAD_UP ... DPAD_DOWN");
            telemetry.addData("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", "");
            telemetry.addData("Launcher current speed: ", motorPower);
            telemetry.addData("Turret current position: ", TurretMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}
