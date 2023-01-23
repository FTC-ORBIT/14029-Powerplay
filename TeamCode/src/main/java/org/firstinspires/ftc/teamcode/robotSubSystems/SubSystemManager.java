package org.firstinspires.ftc.teamcode.robotSubSystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;

public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;
    public static RobotState stateBefore = RobotState.TRAVEL;
    public static ElevatorStates elevatorState = ElevatorStates.INTAKE;
    public static ClawState clawState = ClawState.CLOSE;
    public static ArmState armState = ArmState.BACK;
    private static ClawState clawStateDriver = ClawState.CLOSE;
    private static ClawState lastClawState = ClawState.CLOSE;
    private static ArmState armStateDriver = ArmState.BACK;
    private static ElevatorStates elevatorStateFromDriver = ElevatorStates.LOW;
    private static ElevatorStates elevatorStateToggle = ElevatorStates.OVERRIDE;
    public static IntakeState intakeState = IntakeState.STOP;
    private static IntakeState intakeStateFromDriver = IntakeState.STOP;
    private static boolean lastYButtonState;
    private static boolean lastRightBumperButtonState;
    private static boolean lastDPadDownButtonState;
    private static boolean intakeToggleControl = false; // TODO maybe a better name?
    private static boolean clawToggleControl = false; // TODO maybe a better name?
    private static boolean elevatorToggleControl = true; // TODO maybe better name?
    private static boolean lastLeftBumperButtonState;
    private static boolean clawStateDepleteControl = true;
    private static boolean lastGamePieceState = false;
    private static boolean readyToMoveFromTravelToDeplate = false;
    private static boolean armToggleControl = false;
    private static boolean lastRightStickButtonState = false;
    private static final Delay depleteToIntakeDelay = new Delay(1f);
    private static final Delay travelToDepleteDelay = new Delay(0.3f);
    private static final Delay travelToClawIntakeElevatorDelay = new Delay(1.7f);
    private static final Delay travelToClawIntakeClawDelay = new Delay(0.8f);
    private static final Delay clawIntakeToDepleteDelay = new Delay(0.5f);


    private static RobotState getState(Gamepad gamepad) {
        return gamepad.dpad_up ? RobotState.TRAVEL
                : gamepad.dpad_right ? RobotState.INTAKE
                        : gamepad.dpad_left ? RobotState.CLAWINTAKE : lastState;
    }

    private static RobotState getStateFromWantedAndCurrent(RobotState stateFromDriver){
        switch (stateFromDriver){
            case INTAKE:
                if(GlobalData.hasGamePiece) return RobotState.TRAVEL;
                break;
            case DEPLETE:
                if (clawState.equals(ClawState.OPEN)) GlobalData.hasGamePiece = false;
                if (!GlobalData.hasGamePiece && lastGamePieceState) depleteToIntakeDelay.startAction(GlobalData.currentTime);
                if (depleteToIntakeDelay.isDelayPassed() && Elevator.reachedHeight()){
                    if (stateBefore.equals(RobotState.CLAWINTAKE)) return RobotState.CLAWINTAKE;
                    else return RobotState.INTAKE;
                } else if (!GlobalData.hasGamePiece) elevatorToggleControl = true;
                break;
            case TRAVEL:
                if (readyToMoveFromTravelToDeplate) return RobotState.DEPLETE;
                break;
            case CLAWINTAKE:
                if (GlobalData.hasGamePiece && clawIntakeToDepleteDelay.isDelayPassed()) return RobotState.DEPLETE;
        }
        return stateFromDriver;
    }

    public static void setState(Gamepad gamepad, Gamepad gamepad2, Telemetry telemetry) {
        final RobotState wanted = getStateFromWantedAndCurrent(getState(gamepad));
        if (wanted != null) {
            GlobalData.robotState = wanted;
        }
        setSubsystemToState(gamepad, gamepad2, telemetry);
    }

    public static ElevatorStates getElevatorStateFromDriver (Gamepad gamepad1) {
        return gamepad1.a ? ElevatorStates.LOW
                : gamepad1.b ? ElevatorStates.MID
                        : gamepad1.x ? ElevatorStates.HIGH : null;
    }


    private static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        if (getElevatorStateFromDriver(gamepad1) != null) {
            elevatorStateFromDriver = getElevatorStateFromDriver(gamepad1);
        }
            if (!lastState.equals(GlobalData.robotState)) {
                stateBefore = lastState;
                clawStateDepleteControl = true;
                intakeToggleControl = false;
                clawToggleControl = false;
                elevatorToggleControl = true;
                armToggleControl = false;
                depleteToIntakeDelay.startAction(0);
            }
            switch (GlobalData.robotState) {
                case TRAVEL:
                    intakeState = IntakeState.STOP;
                    if (!Robot.coneDistanceSensor.getState() ) {
                        GlobalData.hasGamePiece = true;
                    }
                    if (stateBefore.equals(RobotState.CLAWINTAKE) && Elevator.reachedHeightVal(ElevatorConstants.lowHeight)) {
                        elevatorState = ElevatorStates.LOW;
                    } else if (GlobalData.hasGamePiece) {
                        clawState = ClawState.CLOSE;
                        if (lastState.equals(RobotState.INTAKE)) travelToDepleteDelay.startAction(GlobalData.currentTime);
                        if (travelToDepleteDelay.isDelayPassed()) elevatorState = elevatorStateFromDriver;
                        if (Elevator.getAndUpdateHeight() >= ElevatorConstants.ableToTurnArmHeight) readyToMoveFromTravelToDeplate = true;
                    } else {
                        elevatorState = ElevatorStates.GROUND;
                        armState = ArmState.BACK;
                        clawState = ClawState.OPEN;
                    }
                    break;
                case INTAKE:
                        if (!Robot.coneDistanceSensor.getState()) {
                            GlobalData.hasGamePiece = true;
                        }
                        elevatorState = ElevatorStates.INTAKE;
                        intakeState = IntakeState.COLLECT;
                        armState = ArmState.BACK;
                    break;
                case CLAWINTAKE:
                    if (stateBefore.equals(RobotState.TRAVEL) || stateBefore.equals(RobotState.INTAKE)) {
                        if (lastState.equals(RobotState.TRAVEL) || lastState.equals(RobotState.INTAKE)) {
                            travelToClawIntakeElevatorDelay.startAction(GlobalData.currentTime);
                            travelToClawIntakeClawDelay.startAction(GlobalData.currentTime);
                            clawState = ClawState.CLOSE;
                        }
                        intakeState = IntakeState.STOP;
                        if (travelToClawIntakeClawDelay.isDelayPassed())
                            elevatorState = ElevatorStates.LOW;
                        if (Elevator.reachedHeightVal(ElevatorConstants.lowHeight) || (elevatorState.equals(ElevatorStates.CLAWINTAKE) && Elevator.reachedHeight())) {
                            armState = ArmState.FRONT;
                            clawState = ClawState.OPEN;
                        }
                        if (travelToClawIntakeElevatorDelay.isDelayPassed())
                            elevatorState = ElevatorStates.CLAWINTAKE;
                    } else {
                        intakeState = IntakeState.STOP;
                        armState = ArmState.FRONT;
                        clawState = ClawState.OPEN;
                        elevatorState = ElevatorStates.CLAWINTAKE;
                    }
                    if (gamepad1.right_bumper && !lastRightBumperButtonState && clawStateDriver.equals(ClawState.OPEN)) GlobalData.hasGamePiece = true;
                    if (!lastGamePieceState && GlobalData.hasGamePiece) clawIntakeToDepleteDelay.startAction(GlobalData.currentTime);
                    break;
                case DEPLETE:
                    readyToMoveFromTravelToDeplate = false;
                    elevatorState = elevatorStateFromDriver;
                    intakeState = IntakeState.STOP;
                    clawState = clawStateDepleteControl ? ClawState.CLOSE : ClawState.OPEN;
                    armState = ArmState.FRONT;
                    if (gamepad1.y) {
                        elevatorToggleControl = false;
                        elevatorStateToggle = ElevatorStates.DEPLETE;
                    }
                    if (!gamepad1.y && lastYButtonState) clawStateDepleteControl = false;
                    break;
            }

        if ((elevatorState.equals(ElevatorStates.GROUND) || elevatorState.equals(ElevatorStates.INTAKE)) && !GlobalData.hasGamePiece){
            if (Elevator.reachedHeight()) clawState = ClawState.OPEN;
                else clawState = ClawState.CLOSE;
        }

        if (!lastDPadDownButtonState && gamepad1.dpad_down){
            armStateDriver = armStateDriver.equals(ArmState.BACK) ? ArmState.FRONT : ArmState.BACK;
            armToggleControl = true;
        }

        if (!lastRightBumperButtonState && gamepad1.right_bumper){
            clawStateDriver = clawStateDriver.equals(ClawState.OPEN) ? ClawState.CLOSE : ClawState.OPEN;
            clawToggleControl = true;
        }

        if (!lastLeftBumperButtonState && gamepad1.left_bumper){
            if (intakeStateFromDriver.equals(IntakeState.COLLECT)) intakeStateFromDriver = IntakeState.DEPLETE;
            else if (intakeStateFromDriver.equals(IntakeState.DEPLETE)) intakeStateFromDriver = IntakeState.STOP;
            else intakeStateFromDriver = IntakeState.COLLECT;
            intakeToggleControl = true;
        }

        if (gamepad1.right_stick_button && !lastRightStickButtonState) {
            elevatorToggleControl = false;
            elevatorStateToggle = ElevatorStates.OVERRIDE;
        }

        if (armToggleControl) armState = armStateDriver;
        if (clawToggleControl) clawState = clawStateDriver;
        if (intakeToggleControl) intakeState = intakeStateFromDriver;

        if (elevatorToggleControl) Elevator.operateTeleop(elevatorState, gamepad1, telemetry);
        else Elevator.operateTeleop(elevatorStateToggle, gamepad1, telemetry);

        Intake.operate(intakeState);
        Claw.operate(clawState);
        Arm.operate(armState);


        lastRightBumperButtonState = gamepad1.right_bumper;
        lastLeftBumperButtonState = gamepad1.left_bumper;
        lastYButtonState = gamepad1.y;
        lastDPadDownButtonState = gamepad1.dpad_down;
        lastGamePieceState = GlobalData.hasGamePiece;
        lastRightStickButtonState = gamepad1.right_stick_button;
        lastClawState = clawState;
        lastState = GlobalData.robotState;
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", GlobalData.robotState);
        telemetry.addData("armState", armState);
        telemetry.addData("clawState", clawState);
        telemetry.addData("elevatorState", elevatorState);
        telemetry.addData("intakeState", intakeState);
        telemetry.addData("delAY", travelToClawIntakeElevatorDelay.isDelayPassed());
    }
}
