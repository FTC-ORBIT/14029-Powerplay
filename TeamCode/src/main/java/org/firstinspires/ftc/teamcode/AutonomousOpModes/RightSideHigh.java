package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import static org.firstinspires.ftc.teamcode.OpenCV.AprilTag.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.AprilTag;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous (group = "rightSideHighCycles")
public class RightSideHigh extends LinearOpMode {
    public static double strafeLeftFirst = 0.4;
    public static double moveForwardToHighX = 1.03;
    public static double turnAngle = Math.toRadians(-90);
    public static double depleteY = 0.6;
    public static double prepareToParkX = 0.65;
    public static double prepareToParkY = 0.45;
    public static double signalSleeveNum = 0;
    public static double park0Distance = 0.03;
    public static double park1Distance = 0.62;
    public static double park2Distance = 1.2;

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTag.init(hardwareMap);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(AprilTag.aprilTagDetectionPipeline);
                FtcDashboard.getInstance().startCameraStream(camera, 60);
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("place", AprilTag.aprilTagDetectionPipeline.getLatestDetections());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevator.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Arm.init(hardwareMap);
        Claw.init(hardwareMap);

        TrajectorySequence firstCone = drive.trajectorySequenceBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
                    Claw.operate(ClawState.CLOSE);
                })
                .strafeLeft(strafeLeftFirst)
                .forward(moveForwardToHighX)
                .turn(turnAngle)
                .back(depleteY - strafeLeftFirst)
                .build();

        Trajectory backFromJunction = drive.trajectoryBuilder(firstCone.end())
                .addDisplacementMarker(() -> {
                    Arm.operate(ArmState.BACK);
                    Claw.operate(ClawState.CLOSE);
                })
                        .forward(depleteY - strafeLeftFirst)
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
        signalSleeveNum = AprilTag.currentTagId(telemetry);
        drive.followTrajectorySequence(firstCone);
        Elevator.height = Drivetrain.motors[1].getCurrentPosition();
        while (!Elevator.reachedHeightVal(ElevatorConstants.highHeight)){
            Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry);
            if (Elevator.height > ElevatorConstants.ableToTurnArmHeight) Arm.operate(ArmState.FRONT);
            drive.update();
        }
        sleep(800);
        Claw.operate(ClawState.OPEN);
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
