package org.firstinspires.ftc.teamcode.robotSubSystems;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class SubSystemManager {

    public static RobotState getStateFromJoystick(final boolean[] buttons) {
        // TODO: adjust to ftc
        // return (buttons[HardwareMap.psCircleIndex]) ? RobotState.TRAVEL
        // : (buttons[HardwareMap.psXIndex]) ? RobotState.INTAKE
        // : (buttons[HardwareMap.psOptionsIndex] || GlobalData.isPOVPressed) ?
        // RobotState.CLIMB
        // : GlobalData.robotState;
        return RobotState.TRAVEL;
    }

    private static RobotState getStateFromWantedAndCurrent(final boolean[] buttons,
                                                           final RobotState robotStateFromJoystick) {
        RobotState state_W = robotStateFromJoystick;
        switch (GlobalData.robotState) {
            case INTAKE:
                break;

            default:
                break;

        }
        return state_W;
    }
        public static RobotState getRobotState ( final RobotState userWantedState){
            return getStateFromWantedAndCurrent(GlobalData.driverPressedButtons, userWantedState);
        }

    private static RobotState prevRobotState = GlobalData.robotState;

    public static void setSubsystemToState(final RobotState robotState) {
        switch (robotState) {
            case TRAVEL:
            default:
                // Intake.set(IntakeState.CLOSED);
                break;

            case INTAKE:
                // Intake.set(IntakeState.COLLECT);
                break;
        }

        // // Set the state of the LED for telemetry.
        // LED.setState();

        prevRobotState = robotState;
    }
}
