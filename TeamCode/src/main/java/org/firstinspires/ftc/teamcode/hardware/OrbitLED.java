package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class OrbitLED {

    private static RevBlinkinLedDriver blinkin;
    private static RevBlinkinLedDriver.BlinkinPattern pattern;
    private static ElapsedTime elapsedTime = new ElapsedTime();

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
            switch (GlobalData.robotState){
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
