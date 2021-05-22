package org.firstinspires.ftc.teamcode.libraries.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class WheelMotors {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public List<DcMotor> initWheels (HardwareMap hardwareMap,
                                    DcMotor.ZeroPowerBehavior zeroPowerBehavior,
                                    DcMotorSimple.Direction direction,
                                    DcMotor.RunMode runMode) {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);

        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);

        frontRight.setDirection(direction);
        backRight.setDirection(direction);

        if(direction == DcMotorSimple.Direction.FORWARD) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        List<DcMotor> wheelMotors = new ArrayList<>();
        wheelMotors.add(frontLeft);
        wheelMotors.add(frontRight);
        wheelMotors.add(backLeft);
        wheelMotors.add(backRight);

        return wheelMotors;
    }

    public void resetEncoders (List<DcMotor> motors,
                               DcMotor.RunMode runMode){
        for (DcMotor dcMotor: motors) {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(runMode);
        }

    }

    public void resetEncoders (List<DcMotor> motors){
        for (DcMotor dcMotor: motors) {
            DcMotor.RunMode runMode = dcMotor.getMode();
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(runMode);
        }

    }

}
