package org.firstinspires.ftc.teamcode.main.driving;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config

public class DashboardConfig {

    public static double kP = 400;
    public static double kI = 0.8;
    public static double kD = 35;
    public static double kF = 3;
    public static double l_velocity = 1725;

    public static double t_kP = 22.5;
    public static double t_kI = 0.35;
    public static double t_kD = 4.5;
    public static double t_kF = 11;
    public static double t_velocity = 850;


    public static double powershot1 = -15 + 15.75;
    public static double powershot2 = -15 + 15.8 + 7.7;
    public static double powershot3 = -15 + 15.8 + 7.8*2;

}
