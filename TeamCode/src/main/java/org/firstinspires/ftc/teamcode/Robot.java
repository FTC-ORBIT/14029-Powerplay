package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitDistanceSensor;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;

@Config
@TeleOp(name = "main")
public class Robot extends LinearOpMode {

    ElapsedTime time = new ElapsedTime();
    OrbitDistanceSensor distanceSensor;
    OrbitColorSensor colorSensor;

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
        GlobalData.currentTime = 0;
        GlobalData.lastTime = 0;
        GlobalData.deltaTime = 0;

        waitForStart();

        while (!isStopRequested()) {

            GlobalData.currentTime = (float) time.seconds();
            Vector leftStick = new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
            Drivetrain.operate(leftStick, (float) OrbitGyro.getAngle());
            SubSystemManager.setState(gamepad1, gamepad2);

            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;

            GlobalData.lastTime = GlobalData.currentTime;
            telemetry.update();
            telemetry.addData("distance", distanceSensor.getDistance());

            packet.put("distance", distanceSensor.getDistance());
            dashboard.sendTelemetryPacket(packet);
        }
    }

}