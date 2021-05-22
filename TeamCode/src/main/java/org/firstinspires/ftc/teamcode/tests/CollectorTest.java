package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(name = "Collector Test", group = "test")

public class CollectorTest extends LinearOpMode {

    private DcMotor frontRight, frontLeft, collector;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        collector = hardwareMap.get(DcMotor.class, "collector");


        collector.setDirection(DcMotorSimple.Direction.FORWARD);
        boolean i = false;

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Waiting for start", "");
            telemetry.update();
        }

        while(opModeIsActive()){

            frontRight.setPower(-gamepad1.left_stick_y);
            frontLeft.setPower(-gamepad1.left_stick_y);

            if(gamepad1.x){
                if(!i) {
                    i = true;
                    collector.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                else {collector.setDirection(DcMotorSimple.Direction.FORWARD);
                    i = false;
                }
            }
            if(gamepad1.dpad_up && collector.getPower() < 1){
                collector.setPower(collector.getPower() + 0.1);
            }
            else if (gamepad1.dpad_down && collector.getPower() > -1){

                collector.setPower(collector.getPower() - 0.1);
            }

            if(gamepad1.dpad_left){
                collector.setPower(0);
            }
        }
    }
}
