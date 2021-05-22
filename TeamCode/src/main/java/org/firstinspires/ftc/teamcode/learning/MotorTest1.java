package org.firstinspires.ftc.teamcode.learning;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "Luci", group = "learning")
public class MotorTest1 extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("portocaliu");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while  (!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Waiting for start", "");
            telemetry.update();
        }

        boolean isBusy = false;
        int targetPos = 0;

        while (opModeIsActive()){
            /*if (gamepad1.x)
                motor.setPower(0.7);
            else if(!gamepad1.y) motor.setPower(0);*/

            if(!isBusy)
                targetPos = motor.getCurrentPosition();
            if (gamepad1.y && motor.getCurrentPosition() < targetPos+2240) {
                isBusy = true;
                motor.setPower(0.7);
            }
            else if (motor.getCurrentPosition() >= targetPos+2240){
                motor.setPower(0);
                isBusy = false;
            }

            telemetry.addData("Motor position ", motor.getCurrentPosition());
            telemetry.update();

        }

    }
}
