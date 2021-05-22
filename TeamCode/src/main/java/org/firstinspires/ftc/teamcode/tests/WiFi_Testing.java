package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "WiFi Test", group = "test")

public class WiFi_Testing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Hello ", "This is a test.");
            telemetry.addData("Press", " start.");
            telemetry.update();
        }

        while(opModeIsActive() && !isStopRequested()){

            telemetry.addData("Well done.", "You have passed the test.");
            telemetry.addData("Press", " stop.");
            telemetry.update();
        }
    }
}
