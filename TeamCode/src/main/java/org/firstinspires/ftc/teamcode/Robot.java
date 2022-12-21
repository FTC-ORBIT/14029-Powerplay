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
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;

@Config
@TeleOp(name = "main")
public class Robot extends LinearOpMode {

    ElapsedTime robotTime = new ElapsedTime();
    OrbitDistanceSensor distanceSensor;
    OrbitColorSensor colorSensor;
    DigitalChannel clawDistanceSensor;
    public static TelemetryPacket packet;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        clawDistanceSensor = hardwareMap.get(DigitalChannel.class, "clawDistanceSensor");
        clawDistanceSensor.setMode(DigitalChannel.Mode.INPUT);

        robotTime.reset();
        //Drivetrain.init(hardwareMap);
        //OrbitGyro.init(hardwareMap);
        //Elevator.init(hardwareMap);
        Claw.init(hardwareMap);
        //Arm.init(hardwareMap);
        //Intake.init(hardwareMap);
        //OrbitLED.init(hardwareMap);

        GlobalData.inAutonomous = false;
        GlobalData.currentTime = 0;
        GlobalData.lastTime = 0;
        GlobalData.deltaTime = 0;

        waitForStart();

        while (!isStopRequested()) {
           // GlobalData.hasGamePiece = clawDistanceSensor.getState();

            if (gamepad2.right_bumper) OrbitGyro.resetGyro();
            GlobalData.currentTime = (float) robotTime.seconds();
            Vector leftStick = new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            //Drivetrain.operate(leftStick,  gamepad1.right_trigger - gamepad1.left_trigger);
            //SubSystemManager.setState(gamepad1, gamepad2);
           // OrbitLED.operate();
            if (gamepad1.b) Claw.operate(ClawState.OPEN);
            else if (gamepad1.a) Claw.operate(ClawState.CLOSE);

            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;

            telemetry.addData("state", !clawDistanceSensor.getState());

            GlobalData.lastTime = GlobalData.currentTime;
            telemetry.update();
            //SubSystemManager.printStates(telemetry);

        }
    }

}