package org.firstinspires.ftc.teamcode.tests;

import org.firstinspires.ftc.teamcode.libraries.hardware.WebCam;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Camera Test", group = "test")
public class Testing extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        new WebCam().initCamera(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Test program ", "Waiting for start");
            telemetry.update();
        }

        while(opModeIsActive());
    }


}
