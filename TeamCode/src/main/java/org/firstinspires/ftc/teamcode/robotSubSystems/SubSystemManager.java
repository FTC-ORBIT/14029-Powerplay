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
    private static ArmState armStateDriver = ArmState.BACK;
    private static final ElevatorStates elevatorStateOverride = ElevatorStates.OVERRIDE;
    private static ElevatorStates elevatorStateFromDriver = ElevatorStates.LOW;
    public static IntakeState intakeState = IntakeState.STOP;
    private static IntakeState intakeStateFromDriver = IntakeState.STOP;
    private static boolean lastYButtonState;
    private static float elevatorTime = 0;
    private static boolean lastRightBumperButtonState;
    private static boolean lastDPadDownButtonState;
    private static boolean leftBumperControl = false; // TODO maybe a better name?
    private static boolean yButtonControl = false; // TODO maybe a better name?
    private static boolean rightStickControl = false; // TODO maybe better name?
    private static boolean lastLeftBumperButtonState;
    private static boolean clawStateControl = true;
    private static float clawTime = 0;
    private static boolean clawTimeControl = true;
    private static boolean elevatorTimeControl = true;
    private static float elevatorTimeClawIntake = 0;
    private static boolean elevatorTimeClawIntakeControl = true;
    private static float armTimeClawIntake = 0;
    private static boolean armTimeClawIntakeControl = true;
    private static boolean depleteDelay = true;
    private static float depleteTime = 0;
    private static boolean clawIntakeToTravelControl = true;
    private static boolean clawIntakeToTravelTimeControl = true;
    private static float clawIntakeToTravelTime = 0;
    private static float check = 0;

    private static RobotState getState(Gamepad gamepad) {
        return gamepad.dpad_up ? RobotState.TRAVEL
                : gamepad.dpad_right ? RobotState.INTAKE
                        : gamepad.dpad_left ? RobotState.CLAWINTAKE : null;
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
            if (!lastState.equals(GlobalData.robotState)) {
                stateBefore = lastState;
                leftBumperControl = false;
                yButtonControl = false;
                rightStickControl = false;
                elevatorTimeControl = true;
                elevatorTimeClawIntakeControl = true;
                armTimeClawIntakeControl = true;
                depleteDelay = true;
                clawStateControl = true;
                clawTimeControl = true;
                clawIntakeToTravelControl = true;
            }
            switch (GlobalData.robotState) {
                case TRAVEL:
                    intakeState = IntakeState.STOP;
                    if (!Robot.coneDistanceSensor.getState()) {
                        GlobalData.hasGamePiece = true;
                    }
                    if (GlobalData.hasGamePiece) {
                        clawState = ClawState.CLOSE;
                        if (clawTimeControl) {
                            clawTime = GlobalData.currentTime;
                            clawTimeControl = false;
                        }
                        if (GlobalData.currentTime - clawTime >= 0.3) {
                            elevatorState = elevatorStateFromDriver;// TODO need to check the timing when intaking
                            if (elevatorTimeControl) {
                                elevatorTime = GlobalData.currentTime;
                                elevatorTimeControl = false;
                            }
                            if (Elevator.getHeight() >= ElevatorConstants.ableToTurnArmHeight) { //todo check
                                GlobalData.robotState = RobotState.DEPLETE;
                            }
                        }
                    } else if (stateBefore == RobotState.CLAWINTAKE && clawIntakeToTravelControl){
                        armState = ArmState.FRONT;
                        intakeState = IntakeState.STOP;
                        elevatorState = ElevatorStates.LOW;
                        clawState = ClawState.CLOSE;
                        if (Elevator.reachedHeight()){
                            if (clawIntakeToTravelTimeControl) {
                                clawIntakeToTravelTime = GlobalData.currentTime;
                                clawIntakeToTravelTimeControl = false;
                            }
                            else if (GlobalData.currentTime - clawIntakeToTravelTime >= 0.5) clawIntakeToTravelControl = false;
                        }
                    }else {
                        elevatorState = ElevatorStates.GROUND;
                        armState = ArmState.BACK;
                        clawState = ClawState.OPEN;
                    }
                    break;
                case INTAKE:
                    if (!GlobalData.hasGamePiece) {
                        if (!Robot.coneDistanceSensor.getState()) {
                            GlobalData.hasGamePiece = true;
                        }
                        elevatorState = ElevatorStates.INTAKE;
                        intakeState = IntakeState.COLLECT;
                        armState = ArmState.BACK;
                    } else {
                        GlobalData.robotState = RobotState.TRAVEL;
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
                            elevatorState = ElevatorStates.CLAWINTAKE;
                        }
                    }
                    if (gamepad1.left_bumper) clawState =   ClawState.CLOSE;
                    else clawState = ClawState.OPEN;
                    if (!Robot.clawTouchSensor.getState()) GlobalData.hasGamePiece = true;
                    if (GlobalData.hasGamePiece) GlobalData.robotState = RobotState.DEPLETE;
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
                            //
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

        if ((elevatorState.equals(ElevatorStates.GROUND) || elevatorState.equals(ElevatorStates.INTAKE)) && !GlobalData.hasGamePiece && !GlobalData.robotState.equals(RobotState.CLAWINTAKE)){
            if (Elevator.reachedHeight()) clawState = ClawState.OPEN;
                else clawState = ClawState.CLOSE;
        }

        if (gamepad1.right_stick_button) {
            rightStickControl = !rightStickControl;
        }

//        if (gamepad1.left_bumper && !lastLeftBumperButtonState) {
//            if (intakeControl){
//                if (intakeState.equals(IntakeState.COLLECT)) intakeStateFromDriver = IntakeState.STOP;
//                else if (intakeState.equals(IntakeState.STOP)) intakeStateFromDriver = IntakeState.DEPLETE;
//                else if (intakeState.equals(IntakeState.DEPLETE)) intakeStateFromDriver = IntakeState.COLLECT;
//                intakeControl = false;
//            } else {
//                if (intakeStateFromDriver.equals(IntakeState.COLLECT)) intakeStateFromDriver = IntakeState.STOP;
//                else if (intakeStateFromDriver.equals(IntakeState.STOP)) intakeStateFromDriver = IntakeState.DEPLETE;
//                else if (intakeStateFromDriver.equals(IntakeState.DEPLETE)) intakeStateFromDriver = IntakeState.COLLECT;
//            }
//        }

//        if (gamepad1.y && !lastYButtonState) {
//            if (clawStateControl) {
//                clawStateDriver = clawState == ClawState.CLOSE ? ClawState.OPEN : ClawState.CLOSE;
//                clawStateControl = false;
//            } else clawStateDriver = clawStateDriver == ClawState.CLOSE ? ClawState.OPEN : ClawState.CLOSE;
//        }

//        if (gamepad1.dpad_down && !lastDPadDownButtonState){
//            if (armToggelControl){
//                armStateDriver = armState == ArmState.FRONT ? ArmState.BACK : ArmState.FRONT;
//                armToggelControl = false;
//            } else armStateDriver = armStateDriver == ArmState.FRONT ? ArmState.BACK ? ArmState.FRONT;
//        }

        //if (rightStickControl) Elevator.operate(elevatorState, gamepad1, telemetry);
        Elevator.operate(elevatorState, gamepad1, telemetry);

        /*if (intakeControl)*/ Intake.operate(intakeState);
//        else Intake.operate(intakeStateFromDriver);

        /*if (clawStateControl)*/  Claw.operate(clawState);
//        else Claw.operate(clawState);


        telemetry.addData("stateBefore", stateBefore);
        telemetry.addData("check", clawIntakeToTravelControl);
        Arm.operate(armState);


        lastRightBumperButtonState = gamepad1.right_bumper;
        lastLeftBumperButtonState = gamepad1.left_bumper;
        lastYButtonState = gamepad1.y;
        lastDPadDownButtonState = gamepad1.dpad_down;
        lastState = GlobalData.robotState;
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("GlobalData.robotState", GlobalData.robotState);
        telemetry.addData("armState", armState);
        telemetry.addData("clawState", clawState);
        telemetry.addData("elevatorState", elevatorState);
        telemetry.addData("intakeState", intakeState);
    }
}
