package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;

public class OrbitLED {

    public static RevBlinkinLedDriver blinkin;
    public static RevBlinkinLedDriver.BlinkinPattern pattern;
    private static ElapsedTime elapsedTime;

    public static void init(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class,"LED");
    }

    public static void operate(RobotState state) {
        elapsedTime.reset();
        if (GlobalData.hasGamePiece && elapsedTime.milliseconds() < 200){pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;}
        else if (GlobalData.hasGamePiece){pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW;}
        else if (state == RobotState.TRAVEL){pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;}
        else if (state == RobotState.INTAKE){pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;}
        else if (state == RobotState.DEPLETE){pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;}

        blinkin.setPattern(pattern);
        }
    }
