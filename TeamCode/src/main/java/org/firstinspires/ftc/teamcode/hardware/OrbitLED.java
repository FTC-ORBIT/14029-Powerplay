package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;

public class OrbitLED {

    public static RevBlinkinLedDriver blinkin;
    public static RevBlinkinLedDriver.BlinkinPattern pattern;
    private static ElapsedTime elapsedTime;

    public static void init(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class,"LED");
        elapsedTime.reset();
    }

    public static void operate() {
        if (GlobalData.hasGamePiece){
            if (elapsedTime.milliseconds() < 200){
                pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW;
            }
        } else {
            elapsedTime.reset();
            switch (SubSystemManager.state){
                case TRAVEL:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                    break;
                case INTAKE:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    break;
                case DEPLETE:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                    break;
            }
        }
        
        blinkin.setPattern(pattern);
        }
    }
