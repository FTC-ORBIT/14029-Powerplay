package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import static org.firstinspires.ftc.teamcode.OpenCV.AprilTag.camera;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.AprilTag;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
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

import java.util.IllegalFormatCodePointException;

@Config
@Autonomous (group = "LeftSideHigh")
public class LeftSideHigh extends LinearOpMode {

    public static double strafeRightDistance = 0.2;
    public static double forwardAndTurnMedium1X = 0.8;
    public static double forwardAndTurnMedium1Y = -0.3;
    public static double forwardAndTurnMedium1TurnAngle = Math.toRadians(-90);
    public static Pose2d forwardAndTurnMediumPose = new Pose2d(forwardAndTurnMedium1X, forwardAndTurnMedium1Y, forwardAndTurnMedium1TurnAngle);
    public static double goToDepleteMedium = 0.2;
    public static double awayFromJunctionMedium = goToDepleteMedium;
    public static double waitTimeDeplete = 400;
    public static double goToConeStacks1X = 1.2;
    public static double goToConeStacks1Y = 0.57;
    public static double goToConeStacks1Angle = Math.toRadians(25);
    public static double waitIntake = 300;
    public static double waitTillElevatorCanGoUp = 150;
    public static double forwardAndTurnHigh1X = 1.2;
    public static double forwardAndTurnHigh1Y = 0.5;
    public static double forwardAndTurnHigh1Angle = Math.toRadians(90);
    public static double goToDepleteHigh1 = 0.2;
    public static double goToConeStacks2X = 1.2;
    public static double goToConeStacks2Y = 0.57;
    public static double goToConeStacks2Angle = Math.toRadians(-90);
    public static double forwardAndTurnHigh2X = 1.2;
    public static double forwardAndTurnHigh2Y = 0.5;
    public static double forwardAndTurnHigh2Angle = Math.toRadians(90);
    public static double goToDepleteHigh2 = 0.2;
    public static double goToConeStacks3X = 1.2;
    public static double goToConeStacks3Y = 0.57;
    public static double goToConeStacks3Angle = Math.toRadians(-90);
    public static double forwardAndTurnHigh3X = 1.2;
    public static double forwardAndTurnHigh3Y = 0.5;
    public static double forwardAndTurnHigh3Angle = Math.toRadians(90);
    public static double goToDepleteHigh3 = 0.2;
    public static double parking0Distance = 0.8;
    public static double parking1Distance = 0.4;
    public static double parking2Distance = 0.2;
    public static double signalSleeveNum = 0;
    public static ElevatorStates elevatorStates = ElevatorStates.GROUND;
    public static ArmState armState = ArmState.BACK;
    public static ClawState clawState = ClawState.CLOSE;
    public static IntakeState intakeState = IntakeState.STOP;

    enum STEPS {
        STRAFE_RIGHT,

        FORWARD_AND_TURN_MEDIUM_1,

        GO_TO_DEPLETE_MEDIUM,

        WAIT_DEPLETE_MEDIUM,

        AWAY_FROM_JUNCTION_MEDIUM,

        GO_TO_CONE_STACKS_1,

        WAIT_INTAKE_1,

        FORWARD_AND_TURN_HIGH_1,

        GO_TO_DEPLETE_HIGH_1,

        WAIT_DEPLETE_HIGH_1,

        AWAY_FROM_JUNCTION_HIGH_1,

        GO_TO_CONE_STACKS_2,

        WAIT_INTAKE_2,

        FORWARD_AND_TURN_HIGH_2,

        GO_TO_DEPLETE_HIGH_2,

        WAIT_DEPLETE_HIGH_2,

        AWAY_FROM_JUNCTION_HIGH_2,

        GO_TO_CONE_STACKS_3,

        WAIT_INTAKE_3,

        FORWARD_AND_TURN_HIGH_3,

        GO_TO_DEPLETE_HIGH_3,

        WAIT_DEPLETE_HIGH_3,

        AWAY_FROM_JUNCTION_HIGH_3,

        GO_TO_PARKING
    }

    STEPS currentStep = STEPS.STRAFE_RIGHT;
    Pose2d startPose = new Pose2d(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException {

        AprilTag.init(hardwareMap);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(AprilTag.aprilTagDetectionPipeline);
                FtcDashboard.getInstance().startCameraStream(camera, 60);
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("place", AprilTag.aprilTagDetectionPipeline.getLatestDetections());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        camera.closeCameraDevice();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevator.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Arm.init(hardwareMap);
        Claw.init(hardwareMap);
        Intake.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        ElapsedTime time = new ElapsedTime();
        time.reset();

        drive.setPoseEstimate(startPose);

        Trajectory strafeRight = drive.trajectoryBuilder(startPose)
                .strafeRight(strafeRightDistance)
                .build();

        Trajectory forwardAndTurnMedium1 = drive.trajectoryBuilder(strafeRight.end())
                .lineToLinearHeading(forwardAndTurnMediumPose)
                .build();

        Trajectory goToDepleteMedium1 = drive.trajectoryBuilder(forwardAndTurnMedium1.end())
                .forward(goToDepleteMedium)
                .build();

        Trajectory awayFromJunctionMedium1 = drive.trajectoryBuilder(goToDepleteMedium1.end())
                .back(awayFromJunctionMedium)
                .build();

        Trajectory goToConeStacksFirst = drive.trajectoryBuilder(awayFromJunctionMedium1.end())
                .lineToLinearHeading(new Pose2d(goToConeStacks1X, goToConeStacks1Y, goToConeStacks1Angle))
                .build();

        Trajectory forwardAndTurnHigh1 = drive.trajectoryBuilder(goToConeStacksFirst.end())
                .lineToLinearHeading(new Pose2d(forwardAndTurnHigh1X, forwardAndTurnHigh1Y, forwardAndTurnHigh1Angle))
                .build();

        Trajectory goToDeplete1High = drive.trajectoryBuilder(forwardAndTurnHigh1.end())
                .forward(goToDepleteHigh1)
                .build();

        Trajectory awayFromJunctionHigh1 = drive.trajectoryBuilder(goToDeplete1High.end())
                .back(goToDepleteHigh1)
                .build();

        Trajectory goToConeStacks2 = drive.trajectoryBuilder(awayFromJunctionHigh1.end())
                .lineToLinearHeading(new Pose2d(goToConeStacks2X, goToConeStacks2Y, goToConeStacks2Angle))
                .build();

        Trajectory forwardAndTurnHigh2 = drive.trajectoryBuilder(goToConeStacks2.end())
                .lineToLinearHeading(new Pose2d(forwardAndTurnHigh2X, forwardAndTurnHigh2Y, forwardAndTurnHigh2Angle))
                .build();

        Trajectory goToDeplete2High = drive.trajectoryBuilder(forwardAndTurnHigh2.end())
                .forward(goToDepleteHigh2)
                .build();

        Trajectory awayFromJunctionHigh2 = drive.trajectoryBuilder(goToDeplete2High.end())
                .back(goToDepleteHigh2)
                .build();

        Trajectory goToConeStacks3 = drive.trajectoryBuilder(awayFromJunctionHigh2.end())
                .lineToLinearHeading(new Pose2d(goToConeStacks3X, goToConeStacks3Y, goToConeStacks3Angle))
                .build();

        Trajectory forwardAndTurnHigh3 = drive.trajectoryBuilder(goToConeStacks3.end())
                .lineToLinearHeading(new Pose2d(forwardAndTurnHigh3X, forwardAndTurnHigh3Y, forwardAndTurnHigh3Angle))
                .build();

        Trajectory goToDeplete3High = drive.trajectoryBuilder(forwardAndTurnHigh3.end())
                .forward(goToDepleteHigh3)
                .build();

        Trajectory awayFromJunctionHigh3 = drive.trajectoryBuilder(goToDeplete3High.end())
                .back(goToDepleteHigh3)
                .build();

        Trajectory parking0 = drive.trajectoryBuilder(awayFromJunctionHigh3.end())
                .strafeLeft(parking0Distance)
                .build();

        Trajectory parking1 = drive.trajectoryBuilder(awayFromJunctionHigh3.end())
                .strafeLeft(parking1Distance)
                .build();

        Trajectory parking2 = drive.trajectoryBuilder(awayFromJunctionHigh3.end())
                .strafeLeft(parking2Distance)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        signalSleeveNum = AprilTag.currentTagId(telemetry);
        telemetry.addData("tag", AprilTag.currentTagId(telemetry));

        currentStep = STEPS.STRAFE_RIGHT;
        drive.followTrajectoryAsync(strafeRight);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentStep) {
                case STRAFE_RIGHT:
                    clawState = ClawState.CLOSE;
                    if (!drive.isBusy()) {
                        currentStep = STEPS.FORWARD_AND_TURN_MEDIUM_1;
                        elevatorStates = ElevatorStates.MID;
                        drive.followTrajectoryAsync(forwardAndTurnMedium1);
                    }
                    break;
                case FORWARD_AND_TURN_MEDIUM_1:
                    if (!drive.isBusy()) {
                        currentStep = STEPS.GO_TO_DEPLETE_MEDIUM;
                        drive.followTrajectoryAsync(goToDepleteMedium1);
                    }
                    break;
                case GO_TO_DEPLETE_MEDIUM:
                    if (!drive.isBusy()) {
                        currentStep = STEPS.WAIT_DEPLETE_MEDIUM;
                        elevatorStates = ElevatorStates.DEPLETE;
                        intakeState = IntakeState.COLLECT;
                        clawState = ClawState.OPEN;
                        time.reset();
                    }
                    break;
                case WAIT_DEPLETE_MEDIUM:
                    if (time.milliseconds() > waitTimeDeplete) {
                        currentStep = STEPS.AWAY_FROM_JUNCTION_MEDIUM;
                        elevatorStates = ElevatorStates.MID;
                        drive.followTrajectoryAsync(awayFromJunctionMedium1);
                    }
                    break;
                case AWAY_FROM_JUNCTION_MEDIUM:
                    intakeState = IntakeState.DEPLETE;
                    if (!drive.isBusy()) {
                        currentStep = STEPS.GO_TO_CONE_STACKS_1;
                        armState = ArmState.FRONT;
                        elevatorStates = ElevatorStates.CLAWINTAKE;
                        drive.followTrajectoryAsync(goToConeStacksFirst);
                    }
                    break;
                case GO_TO_CONE_STACKS_1:
                    intakeState = IntakeState.STOP;
                    if (!drive.isBusy()) {
                        currentStep = STEPS.WAIT_INTAKE_1;
                        clawState = ClawState.CLOSE;
                        time.reset();
                    }
                    break;
                case WAIT_INTAKE_1:
                    if (time.milliseconds() > waitTillElevatorCanGoUp) {
                        elevatorStates = ElevatorStates.HIGH;
                    }
                    if (time.milliseconds() > waitIntake) {
                        currentStep = STEPS.FORWARD_AND_TURN_HIGH_1;
                        armState = ArmState.BACK;
                        drive.followTrajectoryAsync(forwardAndTurnHigh1);
                    }
                    break;
                case FORWARD_AND_TURN_HIGH_1:
                    if (!drive.isBusy()) {
                        currentStep = STEPS.GO_TO_DEPLETE_HIGH_1;
                        drive.followTrajectoryAsync(goToDeplete1High);
                    }
                    break;
                case GO_TO_DEPLETE_HIGH_1:
                    intakeState = IntakeState.COLLECT;
                    if (!drive.isBusy()) {
                        currentStep = STEPS.WAIT_DEPLETE_HIGH_1;
                        clawState = ClawState.OPEN;
                        elevatorStates = ElevatorStates.DEPLETE;
                        drive.followTrajectoryAsync(goToDeplete1High);
                        time.reset();
                    }
                    break;
                case WAIT_DEPLETE_HIGH_1:
                    if (time.milliseconds() > waitTimeDeplete) {
                        currentStep = STEPS.AWAY_FROM_JUNCTION_HIGH_1;
                        drive.followTrajectoryAsync(awayFromJunctionHigh1);
                    }
                    break;
                case AWAY_FROM_JUNCTION_HIGH_1:
                    intakeState = IntakeState.DEPLETE;
                    if (!drive.isBusy()) {
                        currentStep = STEPS.GO_TO_CONE_STACKS_2;
                        armState = ArmState.FRONT;
                        elevatorStates = ElevatorStates.CLAWINTAKE;
                        drive.followTrajectoryAsync(goToConeStacks2);
                    }
                    break;
                case GO_TO_CONE_STACKS_2:
                    intakeState = IntakeState.STOP;
                    if (!drive.isBusy()) {
                        currentStep = STEPS.WAIT_INTAKE_2;
                    }
            }

            drive.update();
            Elevator.operateAutonomous(elevatorStates, telemetry);
            Intake.operate(intakeState);
            Claw.operate(clawState);
            Arm.operate(armState);
            telemetry.addData("isbusy", drive.isBusy());
            telemetry.addData("cuurentState", currentStep);
            telemetry.update();
        }
    }
}

