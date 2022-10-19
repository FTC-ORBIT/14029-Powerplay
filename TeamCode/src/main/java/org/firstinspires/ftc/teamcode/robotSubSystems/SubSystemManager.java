package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotData.GlobalData.isGamePiece;
import static org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw.clawServo;

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
    static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    static ClawState clawState = ClawState.CLOSE;
    static ArmState armState = ArmState.BACK;
    static IntakeState intakeState = IntakeState.STOP;

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
        return elevatorState = gamepad.a ? ElevatorStates.GROUND : gamepad.b ? ElevatorStates.LOW : gamepad.x ? ElevatorStates.MID : gamepad.y ? ElevatorStates.HIGH : null;
    }


    private static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2) {
        elevatorState = getElevatorStateFromSecondDriver(gamepad2);
        switch (state) {
            case TRAVEL:
                intakeState = IntakeState.STOP;
                if (isGamePiece) {
                    clawState = ClawState.CLOSE;
                    if (clawServo.getPosition() == ClawConstants.closed) {
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
                    if (clawServo.getPosition() == ClawConstants.closed) {
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
        if (gamepad1.right_bumper){
            armState = armState == ArmState.FRONT ? ArmState.BACK : ArmState.FRONT;
        }
        Intake.operate(intakeState);
        Elevator.operate(elevatorState);
        Claw.operate(clawState);
        Arm.operate(armState);
    }

}
