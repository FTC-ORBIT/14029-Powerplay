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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTag;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.positionTracker.PoseStorage;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous (group = "LeftSideHigh")
public class LeftSideHigh extends LinearOpMode {
    public static double strafeRightFirst = 0.73;
    public static double moveForwardToHighX = 0.985;
    public static double moveForwardTiHighAngle = Math.toRadians(90);
    public static double goToDepleteHigh = 0.2;
    public static double awayFromHighJunction = 0.25;
    public static double coneStacks1X = 1.335;
    public static double coneStacks1Y = 0.54;
    public static double coneStacksAngle = Math.toRadians(-90);
    public static double theStrafeLeftAfterHighAdditional = 0;
    public static long waitTillIntake = 1000;
    public static double midJunction1X = 1.25;
    public static double midJunction1Y = -0.44;
    public static double midJunctionAngle = Math.toRadians(0);
    public static double goToDepleteJunctionMid1 = 0.125;
    public static double awayFromJunctionMid1 = 0.22;
    public static double coneStacks2X = 1.28;
    public static double coneStacks2Y = 0.51;
    public static double midJunction2X = 1.18;
    public static double midJunction2Y = -0.44;
    public static double goToDepleteJunctionMid2 = 0.05;
    public static double awayFromJunctionMid2 = 0.2;
    public static double coneStacks3X = 1.33;
    public static double coneStacks3Y = 0.5;
    public static double prepareToParkX = 0.65;
    public static double prepareToParkY = -0.8;
    public static double park0Distance = 1.27;
    public static double park1Distance = 0.67;
    public static double park2Distance = 0.05;
    public static double signalSleeveNum = -0.07;


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
        ElapsedTime time = new ElapsedTime();
        time.reset();

        TrajectorySequence firstCone = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(strafeRightFirst)
                .lineToLinearHeading(new Pose2d(moveForwardToHighX, -strafeRightFirst, moveForwardTiHighAngle))
                .build();

        Trajectory goToDepleteHighFirst = drive.trajectoryBuilder(firstCone.end())
                .back(goToDepleteHigh)
                .build();

        Trajectory awayFromHighFirst = drive.trajectoryBuilder(goToDepleteHighFirst.end())
                .forward(awayFromHighJunction)
                .build();

        Trajectory strafeLeftAfterHigh = drive.trajectoryBuilder(awayFromHighFirst.end())
                .lineTo(new Vector2d(coneStacks1X + theStrafeLeftAfterHighAdditional, -strafeRightFirst))
                .build();

        Trajectory goToConeStacks1 = drive.trajectoryBuilder(strafeLeftAfterHigh.end())
                .lineToLinearHeading(new Pose2d(coneStacks1X, coneStacks1Y, coneStacksAngle))
                .build();

        TrajectorySequence goToMediumJunction1 = drive.trajectorySequenceBuilder(goToConeStacks1.end())
                .lineToLinearHeading(new Pose2d(midJunction1X, midJunction1Y, midJunctionAngle))
                .back(goToDepleteJunctionMid1)
                .build();

        Trajectory awayFromMedium1 = drive.trajectoryBuilder(goToMediumJunction1.end())
                .forward(awayFromJunctionMid1)
                .build();

        Trajectory goToConeStacks2 = drive.trajectoryBuilder(awayFromMedium1.end())
                .lineToLinearHeading(new Pose2d(coneStacks2X, coneStacks2Y, coneStacksAngle))
                .build();

        TrajectorySequence goToMediumJunction2 = drive.trajectorySequenceBuilder(goToConeStacks2.end())
                .lineToLinearHeading(new Pose2d(midJunction2X, midJunction2Y, midJunctionAngle))
                .back(goToDepleteJunctionMid2)
                .build();

        Trajectory awayFromMediumJunction2 = drive.trajectoryBuilder(goToMediumJunction2.end())
                .forward(awayFromJunctionMid2)
                .build();

        Trajectory goToConeStacks3 = drive.trajectoryBuilder(awayFromMediumJunction2.end())
                .lineToLinearHeading(new Pose2d(coneStacks3X, coneStacks3Y, coneStacksAngle))
                .build();

        Trajectory parking0 = drive.trajectoryBuilder(new Pose2d())
                .forward(park0Distance)
                .build();
        Trajectory parking1 = drive.trajectoryBuilder(new Pose2d())
                .forward(park1Distance)
                .build();
        Trajectory parking2 = drive.trajectoryBuilder(new Pose2d())
                .forward(park2Distance)
                .build();

        waitForStart();

        signalSleeveNum = AprilTag.currentTagId(telemetry);
        if (!(signalSleeveNum ==0 || signalSleeveNum == 1 || signalSleeveNum == 2)) signalSleeveNum = 0;
        telemetry.addData("tag", AprilTag.currentTagId(telemetry));

        Claw.operate(ClawState.CLOSE);
        sleep(800);
        Elevator.operateAutonomous(ElevatorStates.LOW, telemetry);
        drive.followTrajectorySequence(firstCone);
        Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry);
        Arm.operate(ArmState.FRONT);
        drive.followTrajectory(goToDepleteHighFirst);
        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry);
        Claw.operate(ClawState.OPEN);
        drive.followTrajectory(awayFromHighFirst);
        drive.followTrajectory(strafeLeftAfterHigh);
        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry);
        drive.followTrajectory(goToConeStacks1);
        Claw.operate(ClawState.CLOSE);
        sleep(waitTillIntake);
        Elevator.operateAutonomous(ElevatorStates.MID, telemetry);
        drive.followTrajectorySequence(goToMediumJunction1);
        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry);
        Claw.operate(ClawState.OPEN);
        drive.followTrajectory(awayFromMedium1);
        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry);
        drive.followTrajectory(goToConeStacks2);
        Claw.operate(ClawState.CLOSE);
        sleep(waitTillIntake);
        Elevator.operateAutonomous(ElevatorStates.MID, telemetry);
        drive.followTrajectorySequence(goToMediumJunction2);
        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry);
        Claw.operate(ClawState.OPEN);
        drive.followTrajectory(awayFromMediumJunction2);
        drive.followTrajectory(goToConeStacks3);


//        if (signalSleeveNum == 1) drive.followTrajectory(parking1);
//        else if (signalSleeveNum == 2) drive.followTrajectory(parking2);
//        else if (signalSleeveNum == 0 ) drive.followTrajectory(parking0);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
