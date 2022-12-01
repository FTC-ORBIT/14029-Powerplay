package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitDistanceSensor;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;

@Config
@TeleOp(name = "main")
public class Robot extends LinearOpMode {

    ElapsedTime robotTime = new ElapsedTime();
    OrbitDistanceSensor distanceSensor;
    OrbitColorSensor colorSensor;
    DigitalChannel clawDistanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        //clawDistanceSensor = hardwareMap.get(DigitalChannel.class, "clawDistanceSensor");
        //clawDistanceSensor.setMode(DigitalChannel.Mode.INPUT);

        robotTime.reset();
        Drivetrain.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        //Elevator.init(hardwareMap);
        //Claw.init(hardwareMap);
        //Arm.init(hardwareMap);
        Intake.init(hardwareMap);
        //OrbitLED.init(hardwareMap);

        GlobalData.inAutonomous = false;
        GlobalData.currentTime = 0;
        GlobalData.lastTime = 0;
        GlobalData.deltaTime = 0;

        waitForStart();

        while (!isStopRequested()) {
           // GlobalData.hasGamePiece = clawDistanceSensor.getState();
            
            GlobalData.currentTime = (float) robotTime.seconds();
            Vector leftStick = new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
            Drivetrain.operate(leftStick, (float) OrbitGyro.getAngle());
            SubSystemManager.setState(gamepad1, gamepad2);
           // OrbitLED.operate();

            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;

            GlobalData.lastTime = GlobalData.currentTime;
            telemetry.addData("x", leftStick.x);
            telemetry.addData("y", leftStick.y);
            telemetry.update();
            //SubSystemManager.printStates(telemetry);

            //packet.put("distance", distanceSensor.getDistance());
            //dashboard.sendTelemetryPacket(packet);
        }
    }

}