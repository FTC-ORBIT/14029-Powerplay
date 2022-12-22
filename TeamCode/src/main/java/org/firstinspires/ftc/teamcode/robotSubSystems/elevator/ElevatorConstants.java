package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;

public class ElevatorConstants {
    public static final float intakeHeight = metersToTicks(0f); //TODO change this to the real values
    public static final float groundHeight = metersToTicks(0f);
    public static final float lowHeight = metersToTicks(0f);
    public static final float midHeight = metersToTicks(0f);
    public static final float highHeight = metersToTicks(0f);
    public static final Vector minMetersToTicks = new Vector(0 , 0);
    public static final Vector maxMetersToTicks = new Vector(20000, 2);
    public static final Vector minTicksToMeters = new Vector(minMetersToTicks.y, minMetersToTicks.x);
    public static final Vector maxTicksToMeters = new Vector(maxMetersToTicks.y, maxMetersToTicks.x);
    public static final float kP = (float) (0.85 / 0.93);
    public static final float depletePower = -0.1f;

private static float metersToTicks (float meters){
    return MathFuncs.twoPointsLinear(minMetersToTicks, maxMetersToTicks, meters);
}
}
