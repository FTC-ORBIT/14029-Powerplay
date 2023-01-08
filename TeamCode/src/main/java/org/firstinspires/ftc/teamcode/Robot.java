package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitDistanceSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.OrbitLED;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
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

@Config
@TeleOp(name = "main")
public class Robot extends LinearOpMode {

    ElapsedTime robotTime = new ElapsedTime();
    OrbitDistanceSensor distanceSensor;
    OrbitColorSensor colorSensor;
    public  static DigitalChannel clawDistanceSensor;
    public static TelemetryPacket packet;
    static ElevatorStates states = ElevatorStates.GROUND;
    static  ClawState clawState = ClawState.CLOSE;
    static  ArmState armState = ArmState.BACK;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        clawDistanceSensor = hardwareMap.get(DigitalChannel.class, "clawDistanceSensor");
        clawDistanceSensor.setMode(DigitalChannel.Mode.INPUT);

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
            telemetry.addData("reachedHeight", Elevator.reachedHeight());
        }
    }

    public static  void test(Gamepad gamepad1){
        if(gamepad1.b){
            clawState = ClawState.CLOSE;
        }else if (gamepad1.a){
            clawState = ClawState.OPEN;
        }
        Claw.operate(clawState);

        if (gamepad1.y){
            states = ElevatorStates.LOW;
        }else if(gamepad1.x){
            states = ElevatorStates.GROUND;
        }
//            Elevator.operate(states, gamepad1, telemetry);
        if(gamepad1.right_bumper){
            armState = ArmState.BACK;
        }else if(gamepad1.left_bumper){
            armState = ArmState.FRONT;
        }
    }

}