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
    public static RobotState lastState = RobotState.TRAVEL;
    private static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    private static ClawState clawState = ClawState.CLOSE;
    private static ArmState armState = ArmState.BACK;
    private static ClawState clawStateDriver = ClawState.CLOSE;
    private static ArmState armStateDriver = ArmState.BACK;
    private static IntakeState intakeState = IntakeState.STOP;
    private static boolean lastRightBumperButtonState;
    private static boolean lastYButtonState;
    private static boolean rightBumperControl = false; // TODO maybe a better name?
    private static boolean yButtonControl = false; // TODO maybe a better name?

    private static RobotState getState (Gamepad gamepad){
        return gamepad.b ? RobotState.TRAVEL : gamepad.a ? RobotState.INTAKE : gamepad.x ? RobotState.DEPLETE : gamepad.left_bumper ? RobotState.CLAWINTAKE : null;
    }


    public static void setState (Gamepad gamepad, Gamepad gamepad2){
        final RobotState fromDriver = getState(gamepad);
          if (fromDriver != null){
              state = fromDriver;
          }
          setSubsystemToState(gamepad, gamepad2);
    }

    public static ElevatorStates getElevatorStateFromSecondDriver (Gamepad gamepad){
        return gamepad.a ? ElevatorStates.GROUND : gamepad.b ? ElevatorStates.LOW : gamepad.x ? ElevatorStates.MID : gamepad.y ? ElevatorStates.HIGH : null;
    }

    public static void gamePieceControl (Gamepad gamepad1){
    isGamePiece = gamepad1.dpad_down? true : gamepad1.dpad_left? false : isGamePiece;
    }


    private static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2) {
        final ElevatorStates fromSecondDriver = getElevatorStateFromSecondDriver(gamepad2);
        if (fromSecondDriver != null) {elevatorState = fromSecondDriver;}
        if (lastState != state){
            rightBumperControl = false;
            yButtonControl = false;
        }
        switch (state) {
            case TRAVEL:
                intakeState = IntakeState.STOP;
                if (isGamePiece) {
                    clawState = ClawState.CLOSE ;
                    if (isClawCorrectPos(ClawConstants.closed)) {
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
                if (!isGamePiece) {
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
                if (!isGamePiece) {
                    clawState = ClawState.OPEN;
                } else {
                    clawState = ClawState.CLOSE;
                    if (isClawCorrectPos(ClawConstants.closed)) {
                        state = RobotState.TRAVEL;
                    }
                }
                break;
            case DEPLETE:
                intakeState = IntakeState.STOP;
                clawState = ClawState.CLOSE;
                armState = ArmState.FRONT;
                if (gamepad1.y){clawState = ClawState.OPEN;}
                if (!isGamePiece){state = RobotState.TRAVEL;}
                break;

        }
        if (gamepad1.right_bumper && !lastRightBumperButtonState){
            armStateDriver = armState == ArmState.FRONT ? ArmState.BACK : ArmState.FRONT;
            rightBumperControl = true;
        }

        if (gamepad1.y && !lastYButtonState){
            clawStateDriver = clawState == ClawState.CLOSE ? ClawState.OPEN : ClawState.CLOSE;
            yButtonControl = true;
        }

        gamePieceControl(gamepad1);

        Intake.operate(intakeState);
        Elevator.operate(elevatorState);
        if (!yButtonControl) { Claw.operate(clawState); } else {Claw.operate(clawStateDriver);}
        if (!rightBumperControl) { Arm.operate(armState); } else { Arm.operate(armStateDriver);}


        lastYButtonState = gamepad1.y;
        lastRightBumperButtonState = gamepad1.right_bumper;
        lastState = state;
    }

}
