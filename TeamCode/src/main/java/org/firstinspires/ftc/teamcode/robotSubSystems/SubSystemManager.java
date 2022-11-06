package org.firstinspires.ftc.teamcode.robotSubSystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;

public class SubSystemManager {

    public static RobotState state = RobotState.TRAVEL;
    public static RobotState lastState = RobotState.TRAVEL;
    private static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    private static ClawState clawState = ClawState.CLOSE;
    private static ArmState armState = ArmState.BACK;
    private static ClawState clawStateDriver = ClawState.CLOSE;
    private static ArmState armStateDriver = ArmState.BACK;
    private static final ElevatorStates elevatorStateOverride = ElevatorStates.OVERRIDE;
    private static ElevatorStates elevatorStateFromSecondDriver;
    private static IntakeState intakeState = IntakeState.STOP;
    private static boolean lastRightBumperButtonState;
    private static boolean lastYButtonState;
    private static boolean rightBumperControl = false; // TODO maybe a better name?
    private static boolean yButtonControl = false; // TODO maybe a better name?
    private static boolean rightStickControl = false; // TODO maybe better name?
    private static ElapsedTime time = new ElapsedTime();

    private static RobotState getState(Gamepad gamepad) {
        time.reset();
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                        : gamepad.x ? RobotState.DEPLETE : gamepad.left_bumper ? RobotState.CLAWINTAKE : null;
    }

    public static void setState(Gamepad gamepad, Gamepad gamepad2) {
        final RobotState fromDriver = getState(gamepad);
        if (fromDriver != null) {
            state = fromDriver;
        }
        setSubsystemToState(gamepad, gamepad2);
    }

    public static ElevatorStates getElevatorStateFromSecondDriver(Gamepad gamepad2) {
        return gamepad2.a ? ElevatorStates.GROUND
                : gamepad2.b ? ElevatorStates.LOW
                        : gamepad2.x ? ElevatorStates.MID : gamepad2.y ? ElevatorStates.HIGH : null;
    }

    private static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2) {
        if (getElevatorStateFromSecondDriver(gamepad2) != null) {
            elevatorStateFromSecondDriver = getElevatorStateFromSecondDriver(gamepad2);
        }
        if (!lastState.equals(state)) {
            rightBumperControl = false;
            yButtonControl = false;
            rightStickControl = false;
        }
        switch (state) {
            case TRAVEL:
                intakeState = IntakeState.STOP;
                if (GlobalData.hasGamePiece) {
                    clawState = ClawState.CLOSE;
                    if (Claw.isClawCorrectPos(ClawConstants.closed)) {
                        elevatorState = ElevatorStates.LOW; // TODO need to check the timing when intaking
                        armState = ArmState.FRONT;
                    }
                } else {
                    elevatorState = ElevatorStates.INTAKE;
                    clawState = ClawState.OPEN;
                    armState = ArmState.BACK;
                }
                break;
            case INTAKE:
                if (!GlobalData.hasGamePiece) {
                    elevatorState = ElevatorStates.INTAKE;
                    intakeState = IntakeState.COLLECT;
                    clawState = ClawState.OPEN;
                    armState = ArmState.BACK;
                } else {
                    state = RobotState.TRAVEL;
                }
                break;
            case CLAWINTAKE:
                armState = ArmState.FRONT;
                elevatorState = ElevatorStates.INTAKE;
                intakeState = IntakeState.STOP;
                if (!GlobalData.hasGamePiece) {
                    clawState = ClawState.OPEN;
                } else {
                    clawState = ClawState.CLOSE;
                    if (Claw.isClawCorrectPos(ClawConstants.closed)) {
                        state = RobotState.TRAVEL;
                    }
                }
                break;
            case DEPLETE:
                elevatorState = elevatorStateFromSecondDriver;
                intakeState = IntakeState.STOP;
                clawState = ClawState.CLOSE;
                armState = ArmState.FRONT;
                if (gamepad1.y) {
                    clawState = ClawState.OPEN;
                }
                if (!GlobalData.hasGamePiece) {
                    state = RobotState.TRAVEL;
                }
                break;

        }
        if (gamepad1.right_bumper && !lastRightBumperButtonState) {
            armStateDriver = armState == ArmState.FRONT ? ArmState.BACK : ArmState.FRONT;
            rightBumperControl = true;
        }

        if (gamepad1.y && !lastYButtonState) {
            clawStateDriver = clawState == ClawState.CLOSE ? ClawState.OPEN : ClawState.CLOSE;
            yButtonControl = true;
        }

        if (Math.abs(gamepad1.right_stick_y) > 0.2) {
            rightStickControl = true;
        }

        Intake.operate(intakeState);

        if (!yButtonControl) {
            Claw.operate(clawState);
        } else {
            Claw.operate(clawStateDriver);
        }
        if (!rightBumperControl) {
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
        lastRightBumperButtonState = gamepad1.right_bumper;
        lastState = state;
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("robotState", state);
        telemetry.addData("armState", armState);
        telemetry.addData("clawState", clawState);
        telemetry.addData("elevatorState", elevatorState);
        telemetry.addData("intakeState", intakeState);
        telemetry.addData("gamePieceState", GlobalData.hasGamePiece);
    }
}
