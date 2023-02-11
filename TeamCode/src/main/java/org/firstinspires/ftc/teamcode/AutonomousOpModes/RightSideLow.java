package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import static org.firstinspires.ftc.teamcode.OpenCV.AprilTag.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.AprilTag;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.positionTracker.PoseStorage;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous (group = "rightSideLow")
public class RightSideLow extends LinearOpMode {
    public static double strafeLeftFirst = 0.5;
    public static double moveForwardToHighX = 1.03;
    public static double moveForwardToHighY = 0.4;
    public static double turnAngleHigh = Math.toRadians(90);
    public static double goToDepleteHigh = 0.255;
    public static double awayFromHigh = 0.22;
    public static double theStrafeRightAfterHighX = 0.4;
    public static double theStrafeRightAfterHighY = 0.6;
    public static double coneStacks1X = 1.25;
    public static double coneStacks1Y = -0.15;
    public static double coneStacks1Angle = Math.toRadians(90);
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
        camera.closeCameraDevice();
        GlobalData.autonomousSide = true;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevator.initAutonomous(hardwareMap);
        Intake.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Arm.init(hardwareMap);
        Claw.init(hardwareMap);



        TrajectorySequence firstCone = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(strafeLeftFirst)
                .lineToLinearHeading(new Pose2d(moveForwardToHighX, moveForwardToHighY, turnAngleHigh))
                .build();

        Trajectory goToDepleteHigh1 = drive.trajectoryBuilder(firstCone.end())
                .forward(goToDepleteHigh)
                .build();

        Trajectory awayFromJunctionHigh1 = drive.trajectoryBuilder(goToDepleteHigh1.end())
                .back(awayFromHigh)
                .build();

        Trajectory strafeRightAfterHigh = drive.trajectoryBuilder(awayFromJunctionHigh1.end())
                .lineToLinearHeading(new Pose2d(moveForwardToHighX + theStrafeRightAfterHighX, theStrafeRightAfterHighY, coneStacks1Angle))
                                .build();

        Trajectory goToConeStacks1 = drive.trajectoryBuilder(strafeRightAfterHigh.end())
                        .lineToLinearHeading(new Pose2d(coneStacks1X, coneStacks1Y, coneStacks1Angle))
                                .build();

        waitForStart();
        signalSleeveNum = AprilTag.currentTagId(telemetry);
        if (!(signalSleeveNum ==0 || signalSleeveNum == 1 || signalSleeveNum == 2)) signalSleeveNum = 0;
        Claw.operate(ClawState.CLOSE);
        sleep(500);
        Elevator.operateAutonomous(ElevatorStates.LOW, telemetry, opModeIsActive());
        drive.followTrajectorySequence(firstCone);
        Intake.operate(IntakeState.COLLECT);
        Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry, opModeIsActive());
        drive.followTrajectory(goToDepleteHigh1);
        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry, opModeIsActive());
        Claw.operate(ClawState.OPEN);
        sleep(800);
        Intake.operate(IntakeState.DEPLETE);
        drive.followTrajectory(awayFromJunctionHigh1);
        Intake.operate(IntakeState.STOP);
        drive.followTrajectory(strafeRightAfterHigh);
        Arm.operate(ArmState.FRONT);
        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry, opModeIsActive());
        drive.followTrajectory(goToConeStacks1);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
