package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.groundHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.highHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.intakeHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.kD;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.kP;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.lowHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.midHeight;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.OrbitUtils.PID;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

public class Elevator {
    private static DcMotorEx elevatorMotor;
    private static final PID elevatorPID = new PID(kP, 0, kD, 0, 0);
    private static float lastStateHeight;
    private static boolean ableToChangeConeStacksFloor = false;
    private static float elevatorPower = 0f;
    private static  float wanted = 0;
    public static  float height = 0;
    private static  int coneStacksFloor = 5;
    public static void init(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void operateTeleop(ElevatorStates elevatorState, Gamepad gamepad1, Telemetry telemetry) {
        height = Drivetrain.motors[1].getCurrentPosition();

        switch (elevatorState) {
            case INTAKE:
                wanted = intakeHeight;
                break;
            case GROUND:
                wanted = groundHeight;
                break;
            case LOW:
                wanted = lowHeight;
                break;
            case MID:
                wanted = midHeight;
                break;
            case HIGH:
                wanted = highHeight;
                break;
            case CLAWINTAKE:
                wanted = ElevatorConstants.coneStacksHeight - ElevatorConstants.coneStacksDifference * (5 - coneStacksFloor);
                if ((-gamepad1.right_stick_y > 0.8) && ableToChangeConeStacksFloor) {
                    coneStacksFloor += 1;
                    ableToChangeConeStacksFloor = false;
                }
                else if ((-gamepad1.right_stick_y < -0.8) && ableToChangeConeStacksFloor) {
                    coneStacksFloor -= 1;
                    ableToChangeConeStacksFloor = false;
                } else if (gamepad1.right_stick_y > -0.2 && gamepad1.right_stick_y < 0.2) ableToChangeConeStacksFloor = true;
            case OVERRIDE:
                elevatorPower = -gamepad1.right_stick_y + ElevatorConstants.constantPower;
                break;
            case DEPLETE:
                elevatorPower = ElevatorConstants.depletePower;
                break;
        }
        elevatorPID.setWanted(wanted);
        if (!elevatorState.equals(ElevatorStates.OVERRIDE) && !elevatorState.equals(ElevatorStates.DEPLETE)) {
          elevatorPower = (float) elevatorPID.update(height);
        }

        elevatorMotor.setPower(elevatorPower + ElevatorConstants.constantPower);
        telemetry.addData("height", height);
        telemetry.addData("reachedHeight", reachedHeight());

    }

    public static void operateAutonomous (ElevatorStates elevatorState, Telemetry telemetry) {
        height = Drivetrain.motors[1].getCurrentPosition();
        switch (elevatorState) {
            case INTAKE:
                wanted = intakeHeight;
                break;
            case GROUND:
                wanted = groundHeight;
                break;
            case LOW:
                wanted = lowHeight;
                break;
            case MID:
                wanted = midHeight;
                break;
            case HIGH:
                wanted = highHeight;
                break;
            case DEPLETE:
                elevatorPower = ElevatorConstants.depletePower;
                break;
        }
        while (!Elevator.reachedHeight()) {
            height = Drivetrain.motors[1].getCurrentPosition();
            elevatorPID.setWanted(wanted);
            if (!elevatorState.equals(ElevatorStates.OVERRIDE) && !elevatorState.equals(ElevatorStates.DEPLETE)) {
                elevatorPower = (float) elevatorPID.update(height);
            }

            elevatorMotor.setPower(elevatorPower + ElevatorConstants.constantPower);
            telemetry.addData("height", height);
            telemetry.addData("reachedHeight", reachedHeight());
        }
    }

    public static float getMotorCurrent(Telemetry telemetry){
        telemetry.addData("elevatorMotorCurrent", elevatorMotor.getCurrent(CurrentUnit.AMPS));
        return (float) elevatorMotor.getCurrent(CurrentUnit.AMPS);
    }

    public static boolean reachedHeight (){
        return height >= wanted - 200 && height <= wanted + 200;
    }

    public static boolean reachedHeightVal (float wantedHeight){
        return height >= wantedHeight - 200 && height <= wantedHeight + 200;
    }
    public static float getError(){
        return wanted - height;
    }

    public static void setPower (float power) { elevatorMotor.setPower(-power);}
    public static float getAndUpdateHeight(){
        height = Drivetrain.motors[1].getCurrentPosition();
        return height;
    }

    public static void testMotors(Telemetry telemetry, Gamepad gamepad){
        if (gamepad.a) elevatorMotor.setPower(0.5);
        telemetry.addData("value", Drivetrain.motors[1].getCurrentPosition());
        telemetry.addData("power", elevatorPower);
    }



}
