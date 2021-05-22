package org.firstinspires.ftc.teamcode.libraries.interfaces;

import android.view.KeyEvent;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public interface GeneralMovement {

    void incrementPower (HardwareDevice hwDevice, Gamepad gamepad, KeyEvent event, double pwrIncrement);
}
