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
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;

@Config
@TeleOp(name = "main")
public class Robot extends LinearOpMode {

    ElapsedTime robotTime = new ElapsedTime();
    OrbitDistanceSensor distanceSensor;
    public static DigitalChannel coneDistanceSensor;
    public static DigitalChannel clawTouchSensor;
    public static TelemetryPacket packet;
    private static boolean lastLeftBumperButtonState = false;
    private static boolean lastDPadDownButtonState = false;
    private static boolean lastYButtonState = false;
    private static ElevatorStates states = ElevatorStates.GROUND;
    public static OrbitColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = new OrbitColorSensor(hardwareMap, "colorSensor");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        coneDistanceSensor = hardwareMap.get(DigitalChannel.class, "clawDistanceSensor");
        coneDistanceSensor.setMode(DigitalChannel.Mode.INPUT);

        clawTouchSensor = hardwareMap.get(DigitalChannel.class, "clawTouchSensor");
        clawTouchSensor.setMode(DigitalChannel.Mode.INPUT);



        robotTime.reset();
        Drivetrain.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        Elevator.init(hardwareMap);
        Claw.init(hardwareMap);
        Arm.init(hardwareMap);
        Intake.init(hardwareMap);
        //OrbitLED.init(hardwareMap);

        GlobalData.inAutonomous = false;
        GlobalData.currentTime = 0;
        GlobalData.lastTime = 0;
        GlobalData.deltaTime = 0;
        GlobalData.robotState = RobotState.TRAVEL;
        GlobalData.hasGamePiece = false;

        waitForStart();

        while (!isStopRequested()) {
           if (gamepad2.right_bumper) OrbitGyro.resetGyro();
           GlobalData.currentTime = (float) robotTime.seconds();
           Vector leftStick = new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y);
           Drivetrain.operate(leftStick,  gamepad1.right_trigger - gamepad1.left_trigger);
           SubSystemManager.setState(  gamepad1, gamepad2, telemetry);


            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;


            GlobalData.lastTime = GlobalData.currentTime;
            telemetry.update();
            SubSystemManager.printStates(telemetry);
            colorSensor.printRGB(telemetry);
            telemetry.addData("lf", Drivetrain.motors[0].getCurrentPosition());
            telemetry.addData("rb", Drivetrain.motors[3].getCurrentPosition());
            telemetry.addData("hasGamePiece", GlobalData.hasGamePiece);

        }
    }



}