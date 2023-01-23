package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;

@Disabled
@TeleOp(group = "TestSystems")
public class TestSystems extends LinearOpMode {
    ElevatorStates elevatorState = ElevatorStates.GROUND;
    boolean lastDPadUp;
    public static DigitalChannel coneDistanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        Claw.init(hardwareMap);
        Intake.init(hardwareMap);
        Arm.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Elevator.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        coneDistanceSensor = hardwareMap.get(DigitalChannel.class, "clawDistanceSensor");
        coneDistanceSensor.setMode(DigitalChannel.Mode.INPUT);


        waitForStart();
        if (gamepad1.a) Claw.operate(ClawState.OPEN);
        else if (gamepad1.b) Claw.operate(ClawState.CLOSE);
        if (gamepad1.x) Arm.operate(ArmState.BACK);
        else if (gamepad1.y) Arm.operate(ArmState.FRONT);
        if (gamepad1.dpad_left) Intake.operate(IntakeState.COLLECT);
        else if (gamepad1.dpad_down) Intake.operate(IntakeState.STOP);
        else if (gamepad1.dpad_right) Intake.operate(IntakeState.DEPLETE);
        if (!lastDPadUp && gamepad1.dpad_up){
            switch (elevatorState){
                case GROUND:
                    elevatorState = ElevatorStates.LOW;
                    break;
                case LOW:
                    elevatorState = ElevatorStates.MID;
                    break;
                case MID:
                    elevatorState = ElevatorStates.HIGH;
                    break;
                case HIGH:
                    elevatorState = ElevatorStates.GROUND;
                    break;
            }
        }
        Elevator.operateTeleop(elevatorState, gamepad1, telemetry);
        if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2 || gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2) Drivetrain.operate(new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.right_trigger - gamepad1.left_trigger);


        telemetry.addData("distanceSensor", coneDistanceSensor);
        telemetry.addData("parallelEncoder", Drivetrain.motors[0].getCurrentPosition());
        telemetry.addData("perpandicularEncoder", Drivetrain.motors[3].getCurrentPosition());
        telemetry.addData("elevatorEncoder", Drivetrain.motors[1].getCurrentPosition());


       lastDPadUp = gamepad1.dpad_up;
    }
}
