package org.firstinspires.ftc.teamcode.robotData;

import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;

public class GlobalData {
    public static final boolean inComp = false;
    public static boolean isGamePiece = false; // removed it just for this branch so I will be able to change this variable in other places too

    public static RobotState robotState = RobotState.TRAVEL;
    public static boolean inAutonomous;
    // Joystick buttons pressed (rising edge)
    public static boolean[] driverPressedButtons = new boolean[15];
    public static boolean[] driverReleasedButtons = new boolean[15];
    public static boolean[] driverRawButtons = new boolean[15];

    public static double[] driverAxes = new double[6];
    public static float currentTime = 0;
    public static float lastTime = 0;
    public static float deltaTime = 0;
    final public static float red = 0;
    final public static float blue = 0;


}

