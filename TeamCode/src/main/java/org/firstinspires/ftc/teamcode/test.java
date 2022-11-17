package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;

@TeleOp(name = "test")
public class test extends LinearOpMode {

    private static ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        time.reset();
        Drivetrain.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        Elevator.init(hardwareMap);
        Claw.init(hardwareMap);
        Arm.init(hardwareMap);
        Intake.init(hardwareMap);

        GlobalData.inAutonomous = false;

        waitForStart();

        while (!isStopRequested()) {

        }
    }


    private static void testMotor(DcMotor motor, float power) {
        motor.setPower(power);
    }

    private static void testServo(Servo servo, float startPos, float endPos) {
        time.reset();
        servo.setPosition(startPos);
        if (time.milliseconds() < 1000) {}
        time.reset();
        servo.setPosition(endPos);
        if (time.milliseconds() < 1000) {}

    }

    private static void subsystemChecks(Gamepad gamepad1, Gamepad gamepad2) {
        time.reset();
        while (time.milliseconds() < 5000) {
            SubSystemManager.state = RobotState.TRAVEL;
            SubSystemManager.setState(gamepad1, gamepad2);
        }
        time.reset();
        while (time.milliseconds() < 5000) {
            SubSystemManager.state = RobotState.INTAKE;
            SubSystemManager.setState(gamepad1, gamepad2);
        }
        time.reset();
        while (time.milliseconds() < 5000) {
            SubSystemManager.state = RobotState.CLAWINTAKE;
            SubSystemManager.setState(gamepad1, gamepad2);
        }
        time.reset();
        while (time.milliseconds() < 5000) {
            SubSystemManager.state = RobotState.DEPLETE;
            SubSystemManager.setState(gamepad1, gamepad2);
        }
    }
}


