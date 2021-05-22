package org.firstinspires.ftc.teamcode.libraries.interfaces;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;

import java.util.List;

public interface GeneralInit {

    /**
     * Returns DcMotor object initialised with parameters passed by the user
     * @param hwMap passed by the OpMode calling this method
     * @param name the name of the motor to be initialised
     * @param zeroBehavior the new behavior of the motor when power zero is applied
     * @param direction the direction to set for this motor
     * @param runMode the new current run mode for this motor
     * @return DcMotor object initialised by initMotor
     */
    DcMotor initMotor (@NotNull HardwareMap hwMap, String name ,DcMotor.ZeroPowerBehavior zeroBehavior,
                              DcMotorSimple.Direction direction, DcMotor.RunMode runMode);

    /**
     * Returns list of type DcMotor. All DcMotors initialised with the same parameters
     * @param hwMap passed by the OpMode calling this method
     * @param motorGroup list of names of the motors to be initialised
     * @param zeroBehavior the new behavior of the motor when power zero is applied
     * @param direction the direction to set for this motor
     * @param runMode the new current run mode for this motor
     * @return list of type DcMotor with all objects initialised
     */
    List<DcMotor> initMotorGroup(@NotNull HardwareMap hwMap, List<String> motorGroup, DcMotor.ZeroPowerBehavior zeroBehavior,
                                 DcMotorSimple.Direction direction, DcMotor.RunMode runMode);

    /**
     * Returns Servo object initialised with parameters passed by the user
     * @param hwMap passed by the OpMode calling this method
     * @param name the name of the servo to be initialised
     * @param direction the direction to set for this servo
     * @param minRange minimum angle at which the servo can be set (between 0 and 1)
     * @param maxRange maximum angle at which the servo can be set (between 0 and 1)
     * @return Servo object initialised by initServo
     */
    Servo initServo (@NotNull HardwareMap hwMap, String name, Servo.Direction direction, double minRange, double maxRange);

    /**
     * Returns Servo object initialised with default range (0 to 180 degrees)
     * and the rest of the parameters passed by the user
     * @param hwMap passed by the OpMode calling this method
     * @param name the name of the servo to be initialised
     * @param direction the direction to set for this servo
     * @return Servo object initialised by initServo
     */
    Servo initServo (@NotNull HardwareMap hwMap, String name, Servo.Direction direction);

    /**
     * Returns CRServo object initialised with parameters passed by the user
     * @param hwMap passed by the OpMode calling this method
     * @param name the name of the servo to be initialised
     * @param direction the direction to set for this servo
     * @return CRServo object initialised by initServo
     */
    CRServo initCRServo (@NotNull HardwareMap hwMap, String name, DcMotorSimple.Direction direction);

}
