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
@Autonomous(group = "LeftSideHigh")
public class LeftSideHigh extends LinearOpMode {
    public static double strafeRightFirst = 0.75;
    public static double moveForwardToHighX = 1.055;
    public static double moveForwardToHighY = -0.61;
    public static double moveForwardTiHighAngle = Math.toRadians(-90);
    public static double goToDepleteHigh = 0.27;
    public static double awayFromHighJunction = 0.22;
    public static double coneStacks1X = 1.25;
    public static double coneStacks1Y = 0.68;
    public static double coneStacksAngle = Math.toRadians(-90);
    public static double theStrafeLeftAfterHighAdditional = 0;
    public static double prepareToHighY = -0.13;
    public static double highJunctionX = 1.62;
    public static double highJunctionY = -0.37;
    public static double highJunctionAngleDegrees = -70;
    public static double highJunctionAngleRadians = Math.toRadians(highJunctionAngleDegrees);
    public static double backFromJunctionHigh = 0.25;;
    public static long waitTillIntake = 800;
    public static double coneStacks2X = 1.25;
    public static double parking0Y = 0.25;
    public static double parking1Y = -0.22;
    public static double parking2Y = -0.8;
    public static int signalSleeveNum = 0;

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
                .forward(goToDepleteHigh)
                .build();

        Trajectory awayFromHighFirst = drive.trajectoryBuilder(goToDepleteHighFirst.end())
                .back(awayFromHighJunction)
                .build();

        Trajectory strafeLeftAfterHigh = drive.trajectoryBuilder(awayFromHighFirst.end())
                .lineTo(new Vector2d(coneStacks1X + theStrafeLeftAfterHighAdditional, -strafeRightFirst))
                .build();

        TrajectorySequence goToConeStacks1 = drive.trajectorySequenceBuilder(strafeLeftAfterHigh.end())
                .lineToLinearHeading(new Pose2d(coneStacks1X, coneStacks1Y, coneStacksAngle))
                .build();

        TrajectorySequence prepareToHigh = drive.trajectorySequenceBuilder(goToConeStacks1.end())
                .lineToLinearHeading(new Pose2d(coneStacks1X, prepareToHighY, coneStacksAngle))
                .lineToLinearHeading(new Pose2d(highJunctionX, prepareToHighY, coneStacksAngle))
                .build();

        Trajectory goToHighJunction = drive.trajectoryBuilder(prepareToHigh.end())
                .lineToLinearHeading(new Pose2d(highJunctionX, highJunctionY, coneStacksAngle))
                                .build();

        Trajectory backFromJunctionHigh1 = drive.trajectoryBuilder(goToHighJunction.end())
                        .back(backFromJunctionHigh)
                                .build();

        Trajectory backFromJunctionHigh2 = drive.trajectoryBuilder(backFromJunctionHigh1.end())
                .strafeRight(0.45)
                .build();


        Trajectory parking0Forward = drive.trajectoryBuilder(backFromJunctionHigh2.end())
                        .lineToLinearHeading(new Pose2d(coneStacks2X, parking0Y, coneStacksAngle))
                                .build();

        TrajectorySequence parking1Forward = drive.trajectorySequenceBuilder(backFromJunctionHigh2.end())
                .lineToLinearHeading(new Pose2d(coneStacks2X, parking1Y, coneStacksAngle))
                .lineToLinearHeading(new Pose2d(coneStacks2X - 0.5, parking1Y, coneStacksAngle))
                                .build();

        Trajectory parking2Forward = drive.trajectoryBuilder(backFromJunctionHigh2.end())
                .lineToLinearHeading(new Pose2d(coneStacks2X, parking2Y, coneStacksAngle))
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
        Intake.operate(IntakeState.COLLECT);
        drive.followTrajectory(goToDepleteHighFirst);
        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry, opModeIsActive());
        Claw.operate(ClawState.OPEN);
        Intake.operate(IntakeState.DEPLETE);
        drive.followTrajectory(awayFromHighFirst);
        drive.followTrajectory(strafeLeftAfterHigh);
        Arm.operate(ArmState.FRONT);
        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry, opModeIsActive());
        drive.followTrajectorySequence(goToConeStacks1);
        Claw.operate(ClawState.CLOSE);
        sleep(waitTillIntake);
        Elevator.operateAutonomous(ElevatorStates.LOW, telemetry, opModeIsActive());
        drive.followTrajectorySequence(prepareToHigh);
        Intake.operate(IntakeState.COLLECT);
        Arm.operate(ArmState.BACK);
        Elevator.operateAutonomous(ElevatorStates.HIGH, telemetry, opModeIsActive());
        drive.followTrajectory(goToHighJunction);
        Elevator.operateAutonomous(ElevatorStates.DEPLETE, telemetry, opModeIsActive());
        Claw.operate(ClawState.OPEN);
        Intake.operate(IntakeState.DEPLETE);
        drive.followTrajectory(backFromJunctionHigh1);
        drive.followTrajectory(backFromJunctionHigh2);
        Arm.operate(ArmState.FRONT);
        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry, opModeIsActive());
        if (signalSleeveNum == 0){
            drive.followTrajectory(parking0Forward);
        } else if (signalSleeveNum == 1){
            drive.followTrajectorySequence(parking1Forward);
        } else if (signalSleeveNum == 2){
            drive.followTrajectory(parking2Forward);
        }
        Elevator.operateAutonomous(ElevatorStates.CLAWINTAKE, telemetry, opModeIsActive());
    }
}
