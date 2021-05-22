package org.firstinspires.ftc.teamcode.main.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.libraries.hardware.WheelMotors;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name = "Parking", group = "main")
public class AutonParking extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private List<DcMotor> driveMotors = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

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

            while(Math.abs(backLeft.getCurrentPosition()) <= 4200){
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
