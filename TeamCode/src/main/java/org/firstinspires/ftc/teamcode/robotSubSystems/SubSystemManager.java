package org.firstinspires.ftc.teamcode.robotSubSystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;

public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;
    private static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    public static ClawState clawState = ClawState.CLOSE;
    private static ArmState armState = ArmState.BACK;
    private static ClawState clawStateDriver = ClawState.CLOSE;
    private static ArmState armStateDriver = ArmState.BACK;
    private static final ElevatorStates elevatorStateOverride = ElevatorStates.OVERRIDE;
    private static ElevatorStates elevatorStateFromDriver = ElevatorStates.LOW;
    private static IntakeState intakeState = IntakeState.STOP;
    private static IntakeState intakeStateFromDriver = IntakeState.STOP;
    private static boolean lastYButtonState;
    private static boolean lastRightBumperButtonState;
    private static boolean lastDPadDownButtonState;
    private static boolean leftBumperControl = false; // TODO maybe a better name?
    private static boolean yButtonControl = false; // TODO maybe a better name?
    private static boolean rightStickControl = false; // TODO maybe better name?
    private static boolean lastLeftBumperButtonState;
    private static boolean depleteControl = false;
    private static boolean clawStateControl = true;
    private static float clawTime = 0;
    private static boolean clawTimeControl = true;
    private static boolean elevatorTimeControl = true;
    private static float elevatorTime = 0;
    private static float elevatorTimeClawIntake = 0;
    private static boolean elevatorTimeClawIntakeControl = true;
    private static float armTimeClawIntake = 0;
    private static boolean armTimeClawIntakeControl = true;
    private static boolean depleteDelay = true;
    private static float depleteTime = 0;
    private static boolean armDelay = true;
    private static float armTime = 0;
    private static boolean intakeControl = true;
    private static boolean condition = false;
    private static boolean robotStateControl = true;
    private static boolean lastStateDepleteControl = false;

    private static RobotState getState(Gamepad gamepad) {
        return gamepad.dpad_up ? RobotState.TRAVEL
                : gamepad.dpad_right ? RobotState.INTAKE
                        : gamepad.left_bumper ? RobotState.INTAKE : gamepad.dpad_left ? RobotState.CLAWINTAKE : null;
    }

    public static void setState(Gamepad gamepad, Gamepad gamepad2, Telemetry telemetry) {
        final RobotState fromDriver = getState(gamepad);
        if (fromDriver != null) {
            GlobalData.robotState = fromDriver;
        }
        setSubsystemToState(gamepad, gamepad2, telemetry);
    }

    public static ElevatorStates getElevatorStateFromDriver (Gamepad gamepad1) {
        return gamepad1.a ? ElevatorStates.LOW
                : gamepad1.b ? ElevatorStates.MID
                        : gamepad1.x ? ElevatorStates.MID : null;
    }


    private static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        if (getElevatorStateFromDriver(gamepad1) != null) {
            elevatorStateFromDriver = getElevatorStateFromDriver(gamepad1);
        }
        telemetry.addData("lastState", lastState);
        telemetry.addData("state", GlobalData.robotState);
            if (!lastState.equals(GlobalData.robotState)) {
                robotStateControl = true;
                leftBumperControl = false;
                yButtonControl = false;
                rightStickControl = false;
                elevatorTimeControl = true;
                elevatorTimeClawIntakeControl = true;
                armTimeClawIntakeControl = true;
                depleteDelay = true;
                armDelay = true;
                intakeControl = true;
                clawStateControl = true;
                clawTimeControl = true;
            }
            if (gamepad1.b) condition = false;
            switch (GlobalData.robotState) {
                case TRAVEL:
                    intakeState = IntakeState.STOP;
                    if (!Robot.clawDistanceSensor.getState()) {
                        GlobalData.hasGamePiece = true;
                    }
                    if (GlobalData.hasGamePiece) {
                        clawState = ClawState.CLOSE;
                        if (clawTimeControl) {
                            clawTime = GlobalData.currentTime;
                            clawTimeControl = false;
                        }
                        if (GlobalData.currentTime - clawTime >= 0.5) {
                            telemetry.addData(String.valueOf(GlobalData.currentTime), clawTime);
                            elevatorState = elevatorStateFromDriver;// TODO need to check the timing when intaking
                            if (elevatorTimeControl) {
                                elevatorTime = GlobalData.currentTime;
                                elevatorTimeControl = false;
                            }
                            if (GlobalData.currentTime - elevatorTime >= 1) { //todo check
                                GlobalData.robotState = RobotState.DEPLETE;
                                robotStateControl = false;
                            }
                        }
                    } else {
                        elevatorState = ElevatorStates.GROUND;
                        armState = ArmState.BACK;
                        clawState = ClawState.OPEN;
                    }
                    if (lastStateDepleteControl) {
                        clawState = ClawState.CLOSE;
                        if (Elevator.reachedHeight()) clawState = ClawState.OPEN;
                    }
                    break;
                case INTAKE:
                    if (!GlobalData.hasGamePiece) {
                        if (!Robot.clawDistanceSensor.getState()) {
                            GlobalData.hasGamePiece = true;
                        }
                        elevatorState = ElevatorStates.INTAKE;
                        intakeState = IntakeState.COLLECT;
                        if (!lastStateDepleteControl) clawState = ClawState.OPEN;
                        armState = ArmState.BACK;
                    } else {
                        GlobalData.robotState = RobotState.TRAVEL;
                        robotStateControl = false;
                    }
                    break;
                case CLAWINTAKE:
                    elevatorState = ElevatorStates.LOW;
                    intakeState = IntakeState.STOP;
                    if (elevatorTimeClawIntakeControl) {
                        elevatorTimeClawIntake = GlobalData.currentTime;
                        elevatorTimeClawIntakeControl = false;
                    }
                    if (GlobalData.currentTime - elevatorTimeClawIntake >= 2) {
                        armState = ArmState.FRONT;
                        if (armTimeClawIntakeControl) {
                            armTimeClawIntake = GlobalData.currentTime;
                            armTimeClawIntakeControl = false;
                        }
                        if (GlobalData.currentTime - armTimeClawIntake >= 1) {
                            elevatorState = ElevatorStates.GROUND;
                        }
                    }
                    clawState = gamepad1.x ? ClawState.CLOSE : ClawState.OPEN;
                    if (gamepad2.left_bumper) GlobalData.hasGamePiece = true;
                    if (GlobalData.hasGamePiece) {
                        GlobalData.robotState = RobotState.DEPLETE;
                        robotStateControl = false;
                    }
                    break;
                case DEPLETE:
                    elevatorState = elevatorStateFromDriver;
                    intakeState = IntakeState.STOP;
                    clawState = clawStateControl ? ClawState.CLOSE : ClawState.OPENDEPLETE;
                    armState = ArmState.FRONT;
                    if (!GlobalData.hasGamePiece) {
                        if (depleteDelay) {
                            depleteTime = GlobalData.currentTime;
                            depleteDelay = false;
                        }
                        if (GlobalData.currentTime - depleteTime >= 1) {
                            GlobalData.robotState = RobotState.INTAKE;
                            robotStateControl = false;
                        }
                    }
                    if (gamepad1.right_bumper && !lastRightBumperButtonState) {
                        clawStateControl = false;
                    }
                    if (SubSystemManager.clawState.equals(ClawState.OPENDEPLETE)) {
                        GlobalData.hasGamePiece = false;
                    }

                    break;
            }

            if ((elevatorState.equals(ElevatorStates.GROUND) || elevatorState.equals(ElevatorStates.INTAKE)) && !GlobalData.hasGamePiece){
                if (Elevator.reachedHeight()) clawState = ClawState.OPEN;
                else clawState = ClawState.CLOSE;
            }

            if (gamepad1.dpad_down && !lastDPadDownButtonState) {
                if (!leftBumperControl) {
                    if (armState.equals(ArmState.FRONT)) {
                        armStateDriver = ArmState.BACK;
                    } else {
                        armStateDriver = ArmState.FRONT;
                    }
                    leftBumperControl = true;
                } else {
                    if (armStateDriver.equals(ArmState.FRONT)) armStateDriver = ArmState.BACK;
                    else armStateDriver = ArmState.FRONT;
                }
            }

            if (gamepad1.y && !lastYButtonState) {
                if (yButtonControl) {
                    clawStateDriver = clawState == ClawState.CLOSE ? ClawState.OPEN : ClawState.CLOSE;
                    yButtonControl = false;
                } else
                    clawStateDriver = clawStateDriver == ClawState.CLOSE ? ClawState.OPEN : ClawState.CLOSE;
            }

            if (gamepad1.right_stick_button) {
                rightStickControl = !rightStickControl;
            }

            if (gamepad1.left_bumper && !lastLeftBumperButtonState) {
                if (intakeControl) {
                    if (intakeState.equals(IntakeState.COLLECT)) {
                        intakeStateFromDriver = IntakeState.DEPLETE;
                    } else if (intakeState.equals(IntakeState.DEPLETE)) {
                        intakeStateFromDriver = IntakeState.STOP;
                    } else {
                        intakeStateFromDriver = IntakeState.COLLECT;
                    }
                    intakeControl = false;
                } else {
                    if (intakeStateFromDriver.equals(IntakeState.COLLECT)) {
                        intakeStateFromDriver = IntakeState.DEPLETE;
                    } else if (intakeStateFromDriver.equals(IntakeState.DEPLETE)) {
                        intakeStateFromDriver = IntakeState.STOP;
                    } else {
                        intakeStateFromDriver = IntakeState.COLLECT;
                    }
                }
            }
            telemetry.addData("clawTimeControl", clawTimeControl);
            telemetry.addData("condition", condition);

            if (intakeControl) Intake.operate(intakeState);
            else Intake.operate(intakeStateFromDriver);

            if (!yButtonControl) {
                Claw.operate(clawState);
            } else {
                Claw.operate(clawStateDriver);
            }
            if (!leftBumperControl) {
                Arm.operate(armState);
            } else {
                Arm.operate(armStateDriver);
            }
            if (!rightStickControl) {
                Elevator.operate(elevatorState, gamepad1);
            } else {
                Elevator.operate(elevatorStateOverride, gamepad1);
            }

            lastYButtonState = gamepad1.y;
            lastLeftBumperButtonState = gamepad1.left_bumper;
            lastDPadDownButtonState = gamepad1.dpad_down;
            lastRightBumperButtonState = gamepad1.right_bumper;
            if (robotStateControl) lastState = GlobalData.robotState;
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", GlobalData.robotState);
        telemetry.addData("armState", armState);
        telemetry.addData("clawState", clawState);
        telemetry.addData("elevatorState", elevatorState);
        telemetry.addData("intakeState", intakeState);
    }
}
