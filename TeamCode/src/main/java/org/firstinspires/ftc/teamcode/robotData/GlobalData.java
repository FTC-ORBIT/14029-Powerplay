package org.firstinspires.ftc.teamcode.robotData;

import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;

public class GlobalData {
    public static final boolean inComp = false;

    public static RobotState robotState = RobotState.TRAVEL;

    // Joystick buttons pressed (rising edge)
    public static boolean[] driverPressedButtons = new boolean[15];
    public static boolean[] driverReleasedButtons = new boolean[15];
    public static boolean[] driverRawButtons = new boolean[15];

    public static double[] driverAxes = new double[6];

    public static boolean isAutonomous;

}
//useless?