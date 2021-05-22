package org.firstinspires.ftc.teamcode.libraries.implementations;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libraries.interfaces.GeneralInit;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

public class GeneralInitImpl implements GeneralInit {

    public DcMotorEx initExMotor(@NotNull HardwareMap hwMap, String name, DcMotor.ZeroPowerBehavior zeroBehavior,
                                 DcMotorSimple.Direction direction, DcMotor.RunMode runMode) {

        DcMotorEx motor = hwMap.get(DcMotorEx.class, name);

        motor.setZeroPowerBehavior(zeroBehavior);
        motor.setDirection(direction);
        motor.setMode(runMode);

        return motor;
    }

    @Override
    public DcMotor initMotor(@NotNull HardwareMap hwMap, String name, DcMotor.ZeroPowerBehavior zeroBehavior,
                             DcMotorSimple.Direction direction, DcMotor.RunMode runMode) {

        DcMotor motor = hwMap.dcMotor.get(name);

        motor.setZeroPowerBehavior(zeroBehavior);
        motor.setDirection(direction);
        motor.setMode(runMode);

        return motor;
    }

    @Override
    public List<DcMotor> initMotorGroup(@NotNull HardwareMap hwMap, List<String> motorGroup, DcMotor.ZeroPowerBehavior zeroBehavior,
                                        DcMotorSimple.Direction direction, DcMotor.RunMode runMode) {

        List<DcMotor> motorList = new ArrayList<>();

        for (String name: motorGroup) {
            DcMotor motor = hwMap.dcMotor.get(name);

            motor.setZeroPowerBehavior(zeroBehavior);
            motor.setDirection(direction);
            motor.setMode(runMode);

            motorList.add(motor);
        }
        return motorList;
    }

    @Override
    public Servo initServo(@NotNull HardwareMap hwMap, String name, Servo.Direction direction, double minRange, double maxRange) {
        Servo servo = hwMap.servo.get(name);

        servo.setDirection(direction);
        servo.scaleRange(minRange, maxRange);

        return servo;
    }

    @Override
    public Servo initServo(@NotNull HardwareMap hwMap, String name, Servo.Direction direction) {
        Servo servo = hwMap.servo.get(name);

        servo.setDirection(direction);
        servo.scaleRange(0, 1);

        return servo;
    }

    @Override
    public CRServo initCRServo(@NotNull HardwareMap hwMap, String name, DcMotorSimple.Direction direction) {
        CRServo crServo = hwMap.crservo.get(name);

        crServo.setDirection(direction);

        return crServo;
    }
}
