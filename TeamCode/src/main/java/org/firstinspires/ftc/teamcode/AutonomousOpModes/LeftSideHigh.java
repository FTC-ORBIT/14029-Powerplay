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
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous (group = "LeftSideHigh")
public class LeftSideHigh extends LinearOpMode {
    public static double strafeRightFirst = 0.75;
    public static double moveForwardToHighX = 1.01;
    public static double moveForwardToHighY = -0.75;
    public static double moveForwardTiHighAngle = Math.toRadians(90);
    public static double goToDepleteHigh = 0.205;
    public static double awayFromHighJunction = 0.22;
    public static double coneStacks1X = 1.3;
    public static double coneStacks1Y = 0.6;
    public static double coneStacksAngle = Math.toRadians(-90);
    public static double theStrafeLeftAfterHighAdditional = -0.04;
    public static long waitTillIntake = 800;
    public static double lowJunction1X = 1;
    public static double lowJunction1Y = 0.17;
    public static double lowJunctionAngle = Math.toRadians(-120);
    public static double awayFromLow = 0.2;
    public static double strafeLeftAfterLow = 0.2;
    public static double minusXConeStacks2 = 0.1;
    public static double coneStacks2X = 1.4;
    public static double coneStacks2Y = 0.6;
    public static double lowJunction2X = 1;
    public static double lowJunction2Y = 0.17;
    public static double awayFromLow2 = 0.45;
    public static double prepareForParkingAngleDegrees = 60;
    public static double prepareForParkingAngleRadians = Math.toRadians(prepareForParkingAngleDegrees);
    public static long waitTillArmReachesPos = 400;
    public static double park0X = 1.27;
    public static double park0Y = 0.3;
    public static double park1X = 1.27;
    public static double park1Y = -0.25;
    public static double park2X = 1.25;
    public static double park2Y = -0.85;
    public static double signalSleeveNum = 0;


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


        GlobalData.autonomousSide = false;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevator.initAutonomous(hardwareMap);
        Drivetrain.init(hardwareMap);
        Intake.init(hardwareMap);
        Arm.init(hardwareMap);
        Claw.init(hardwareMap);
        ElapsedTime time = new ElapsedTime();
        time.reset();

        TrajectorySequence firstCone = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(strafeRightFirst)
                .lineToLinearHeading(new Pose2d(moveForwardToHighX, moveForwardToHighY, moveForwardTiHighAngle))
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

        TrajectorySequence goToConeStacks1 = drive.trajectorySequenceBuilder(strafeLeftAfterHigh.end())
                    .turn(Math.toRadians(190))
                .lineToLinearHeading(new Pose2d(coneStacks1X, coneStacks1Y, coneStacksAngle))
                .build();

        Trajectory goToLowJunction1 = drive.trajectoryBuilder(goToConeStacks1.end())
                .lineToLinearHeading(new Pose2d(lowJunction1X, lowJunction1Y, lowJunctionAngle))
                .build();

        Trajectory awayFromLowJunction = drive.trajectoryBuilder(goToLowJunction1.end())
                .back(awayFromLow)
                .build();

        TrajectorySequence goToConeStacks2 = drive.trajectorySequenceBuilder(goToLowJunction1.end())
                .lineTo(new Vector2d(coneStacks2X - minusXConeStacks2, coneStacks2Y - strafeLeftAfterLow))
                .lineToLinearHeading(new Pose2d(coneStacks2X - minusXConeStacks2, coneStacks2Y, coneStacksAngle))
                .build();

        Trajectory lowJunction2 = drive.trajectoryBuilder(goToConeStacks2.end())
                .lineToLinearHeading(new Pose2d(lowJunction2X, lowJunction2Y, lowJunctionAngle))
                .build();

        TrajectorySequence prepareForParking = drive.trajectorySequenceBuilder(lowJunction2.end())
                .back(awayFromLow2)
                .turn(prepareForParkingAngleRadians)
                .build();


        Trajectory parking0 = drive.trajectoryBuilder(prepareForParking.end())
                .lineToLinearHeading(new Pose2d(park0X, park0Y, coneStacksAngle))
                .build();
        Trajectory parking1 = drive.trajectoryBuilder(prepareForParking.end())
                .lineToLinearHeading(new Pose2d(park1X, park1Y, coneStacksAngle))
                .build();
        Trajectory parking2 = drive.trajectoryBuilder(prepareForParking.end())
                .lineToLinearHeading(new Pose2d(park2X, park2Y, coneStacksAngle))
                .build();

        waitForStart();

        signalSleeveNum = AprilTag.currentTagId(telemetry);
        if (!(signalSleeveNum ==0 || signalSleeveNum == 1 || signalSleeveNum == 2)) signalSleeveNum = 0;
        telemetry.update();
        telemetry.addData("tag", AprilTag.currentTagId(telemetry));
        telemetry.update();

        Claw.operate(ClawState.CLOSE);
        sleep(waitTillIntake);
        Elevator.operateAutonomous(ElevatorStates.LOW, telemetry, opModeIsActive());
        drive.followTrajectorySequence(firstCone);
        Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry, opModeIsActive());
        Arm.operate(ArmState.FRONT);
        drive.followTrajectory(goToDepleteHighFirst);
        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry, opModeIsActive());
        Claw.operate(ClawState.OPEN);
        drive.followTrajectory(awayFromHighFirst);
        drive.followTrajectory(strafeLeftAfterHigh);
        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry, opModeIsActive());
        drive.followTrajectorySequence(goToConeStacks1);
        Claw.operate(ClawState.CLOSE);
        sleep(waitTillIntake);
        Elevator.operateAutonomous(ElevatorStates.LOW, telemetry, opModeIsActive());
        Arm.operate(ArmState.BACK);
        Intake.operate(IntakeState.COLLECT);
        drive.followTrajectory(goToLowJunction1);
        Intake.operate(IntakeState.DEPLETE);
        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry, opModeIsActive());
        Claw.operate(ClawState.OPEN);
        Intake.operate(IntakeState.DEPLETE);
        drive.followTrajectory(awayFromLowJunction);
        Elevator.operateAutonomous(ElevatorStates.MID, telemetry, opModeIsActive());
        Arm.operate(ArmState.FRONT);
        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry, opModeIsActive());
        drive.followTrajectorySequence(goToConeStacks2);
        Intake.operate(IntakeState.STOP);
        Claw.operate(ClawState.CLOSE);
        sleep(waitTillIntake);
        Elevator.operateAutonomous(ElevatorStates.LOW, telemetry, opModeIsActive());
        Arm.operate(ArmState.BACK);
        Intake.operate(IntakeState.COLLECT);
        drive.followTrajectory(lowJunction2);
        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry, opModeIsActive());
        Claw.operate(ClawState.OPEN);
        Intake.operate(IntakeState.DEPLETE);
        drive.followTrajectorySequence(prepareForParking);
        Arm.operate(ArmState.FRONT);
        sleep(waitTillArmReachesPos);
        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry, opModeIsActive());
        if (signalSleeveNum == 1) drive.followTrajectory(parking1);
        else if (signalSleeveNum == 2) drive.followTrajectory(parking2);
        else if (signalSleeveNum == 0 ) drive.followTrajectory(parking0);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
