package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotData.GlobalData.isGamePiece;
import static org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw.clawServo;
import static org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw.isClawCorrectPos;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;

public class SubSystemManager {

    public static RobotState state = RobotState.TRAVEL;
    private static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    private static ClawState clawState = ClawState.CLOSE;
    private static ArmState armState = ArmState.BACK;
    private static IntakeState intakeState = IntakeState.STOP;
    private static boolean armPress = false;
    private static boolean clawPress = false;
    private static boolean stateSwitchToggle = false;
    private static RobotState fromDriver;

    private static RobotState getState(Gamepad gamepad) {
        return gamepad.b ? RobotState.TRAVEL : gamepad.a ? RobotState.INTAKE : gamepad.x ? RobotState.DEPLETE : gamepad.left_bumper ? RobotState.CLAWINTAKE : null;
    }


    public static void setState(Gamepad gamepad, Gamepad gamepad2) {
        fromDriver = getState(gamepad);
        if (fromDriver != null) {
            state = fromDriver;
        }
        setSubsystemToState(gamepad, gamepad2);
    }

    public static ElevatorStates getElevatorStateFromSecondDriver(Gamepad gamepad) {
        return gamepad.a ? ElevatorStates.GROUND : gamepad.b ? ElevatorStates.LOW : gamepad.x ? ElevatorStates.MID : gamepad.y ? ElevatorStates.HIGH : null;
    }


    private static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2) {
        final ElevatorStates fromSecondDriver = getElevatorStateFromSecondDriver(gamepad2);
        if (fromSecondDriver != null) {
            elevatorState = fromSecondDriver;
        }
        switch (state) {
            case TRAVEL:
                if (fromDriver != null || stateSwitchToggle) {
                    intakeState = IntakeState.STOP;
                    if (isGamePiece) {
                        clawState = ClawState.CLOSE;
                        if (isClawCorrectPos(ClawConstants.closed)) {
                            elevatorState = ElevatorStates.LOW; // TODO need to check the timing when intaking
                            armState = ArmState.FRONT;
                        }
                    } else {
                        elevatorState = ElevatorStates.INTAKE;
                        clawState = ClawState.OPEN;
                        armState = ArmState.BACK;
                    }
                }
                break;
            case INTAKE:
                if (fromDriver != null || stateSwitchToggle) {
                    if (!isGamePiece) {
                        elevatorState = ElevatorStates.INTAKE;
                        intakeState = IntakeState.COLLECT;
                        clawState = ClawState.OPEN;
                        armState = ArmState.BACK;
                    } else {
                        state = RobotState.TRAVEL;
                        stateSwitchToggle = true;
                    }
                }
                break;
            case CLAWINTAKE:
                if (fromDriver != null || stateSwitchToggle) {
                    armState = ArmState.FRONT;
                    elevatorState = ElevatorStates.INTAKE;
                    intakeState = IntakeState.STOP;
                    if (!isGamePiece) {
                        clawState = ClawState.OPEN;
                    } else {
                        clawState = ClawState.CLOSE;
                        if (isClawCorrectPos(ClawConstants.closed)) {
                            state = RobotState.TRAVEL;
                            stateSwitchToggle = true;
                        }
                    }
                }
                break;
            case DEPLETE:
                if (fromDriver != null || stateSwitchToggle) {
                    intakeState = IntakeState.STOP;
                    clawState = ClawState.CLOSE;
                    armState = ArmState.FRONT;
                    if (gamepad1.y) {
                        clawState = ClawState.OPEN;
                    }
                    if (!isGamePiece) {
                        state = RobotState.TRAVEL;
                        stateSwitchToggle = true;
                    }
                }
                break;

        }
        if (gamepad1.right_bumper) {
            if (!armPress) {
                armState = armState == ArmState.FRONT ? ArmState.BACK : ArmState.FRONT;
            }
            clawPress = true;
        } else armPress = false;

        if (gamepad1.y) {
            if (!clawPress) {
                clawState = clawState == ClawState.CLOSE ? ClawState.OPEN : ClawState.CLOSE;
            }
            clawPress = true;
        } else clawPress = false;

        Intake.operate(intakeState);
        Elevator.operate(elevatorState);
        Claw.operate(clawState);
        Arm.operate(armState);
        stateSwitchToggle = false;
    }

}
