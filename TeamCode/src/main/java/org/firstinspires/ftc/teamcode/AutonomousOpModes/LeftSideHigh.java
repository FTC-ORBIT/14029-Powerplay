package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants;

@Config
@Autonomous (group = "LeftSideHigh")
public class LeftSideHigh extends LinearOpMode {

    public static double strafeRightFirst = 0.7;
    public static double moveForwardToHighX = 0.97;
    public static double turnAngle = Math.toRadians(90);
    public static double depleteY = 0.87;
    public static double prepareToParkX = 0.65;
    public static double prepareToParkY = -0.8;
    public static double signalSleeveNum = 0;
    public static double park0Distance = 0.03;
    public static double park1Distance = 0.67;
    public static double park2Distance = 1.25;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevator.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Arm.init(hardwareMap);
        Claw.init(hardwareMap);
        ElapsedTime time = new ElapsedTime();

        TrajectorySequence firstCone = drive.trajectorySequenceBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
                    Claw.operate(ClawState.CLOSE);
                })
                .strafeRight(strafeRightFirst)
                .forward(moveForwardToHighX)
                .turn(turnAngle)
                .back(depleteY - strafeRightFirst)
                .build();

        Trajectory backFromJunction = drive.trajectoryBuilder(firstCone.end())
                .addDisplacementMarker(() -> {
                    Arm.operate(ArmState.BACK);
                    Claw.operate(ClawState.CLOSE);
                })
                .forward(depleteY - strafeRightFirst)
                .build();

        Trajectory prepareToParking = drive.trajectoryBuilder(backFromJunction.end())
                .lineTo(new Vector2d(prepareToParkX, prepareToParkY))
                .build();

        Trajectory parking0 = drive.trajectoryBuilder(prepareToParking.end())
                .forward(park0Distance)
                .build();

        Trajectory parking1 = drive.trajectoryBuilder(prepareToParking.end())
                .forward(park1Distance)
                .build();


        Trajectory parking2 = drive.trajectoryBuilder(prepareToParking.end())
                .forward(park2Distance)
                .build();

        waitForStart();

        drive.followTrajectorySequence(firstCone);
        Elevator.height = Drivetrain.motors[1].getCurrentPosition();
        while (!Elevator.reachedHeightVal(ElevatorConstants.highHeight)){
            Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry);
            if (Elevator.height > ElevatorConstants.ableToTurnArmHeight) Arm.operate(ArmState.FRONT);
            drive.update();
        }
        double startTime = time.seconds();
        while (time.seconds() - startTime <= 2) Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry);
        Claw.operate(ClawState.OPEN);
        sleep(800);
        while (!Elevator.reachedHeightVal(ElevatorConstants.highHeight)){
            Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry);
            drive.update();
        }
        sleep(800);
        drive.followTrajectory(backFromJunction);
        while (!Elevator.reachedHeightVal(ElevatorConstants.groundHeight)){
            Elevator.operateAutonomous(ElevatorStates.GROUND, telemetry);
            drive.update();
        }
        drive.followTrajectory(prepareToParking);
        if (signalSleeveNum == 1) drive.followTrajectory(parking1);
        else if (signalSleeveNum == 2) drive.followTrajectory(parking2);
        else if (signalSleeveNum == 0 ) drive.followTrajectory(parking0);
    }
}
