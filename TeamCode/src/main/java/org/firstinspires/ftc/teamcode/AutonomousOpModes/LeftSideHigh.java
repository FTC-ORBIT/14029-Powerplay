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
    public static Pose2d GoToConeStacks1 = new Pose2d(goToConeStacks1X, goToConeStacks1Y, goToConeStacks1Angle);
    public static double waitIntake1 = 1;
    public static double forwardAndTurnHigh1X = 1.2;
    public static double forwardAndTurnHigh1Y = 0.5;
    public static double forwardAndTurnHigh1Angle = Math.toRadians(90);
    public static Pose2d forwardAndTurnHigh1 = new Pose2d(forwardAndTurnHigh1X, forwardAndTurnHigh1Y, forwardAndTurnHigh1Angle);
    public static double goToDepleteHigh1 = 0.2;
    public static Pose2d awayFromJunctionHigh = forwardAndTurnHigh1;
    public static double goToConeStacks2X = 1.2;
    public static double goToConeStacks2Y = 0.57;
    public static double goToConeStacks2Angle = Math.toRadians(-90);
    public static Pose2d GoToConeStacks2 = new Pose2d(goToConeStacks2X, goToConeStacks2Y, goToConeStacks2Angle);
    public static double forwardAndTurnHigh2X = 1.2;
    public static double forwardAndTurnHigh2Y = 0.5;
    public static double forwardAndTurnHigh2Angle = Math.toRadians(90);
    public static Pose2d forwardAndTurnHigh2 = new Pose2d(forwardAndTurnHigh2X, forwardAndTurnHigh2Y, forwardAndTurnHigh2Angle);
    public static double goToDepleteHigh2 = 0.2;
    public static Pose2d awayFromJunctionHigh2 = forwardAndTurnHigh2;
    public static double parking0 = 0.8;
    public static double parking1 = 0.4;
    public static double parking2 = 0.2;
    public static double signalSleeveNum = 0;
    public static ElevatorStates elevatorStates = ElevatorStates.GROUND;
    public static ArmState armState = ArmState.BACK;
    public static ClawState clawState = ClawState.CLOSE;

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

        GO_TO_PARKING

    }

    STEPS currentStep = STEPS.FORWARD_AND_TURN_MEDIUM_1;
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
        ElapsedTime time = new ElapsedTime();
        time.reset();

        drive.setPoseEstimate(startPose);

        Trajectory strafeRight = drive.trajectoryBuilder(startPose)
                .strafeRight(strafeRightDistance)
                .build();

        Trajectory forwardAndTurnMedium1 = drive.trajectoryBuilder(strafeRight.end())
                .lineToSplineHeading(forwardAndTurnMediumPose)
                .build();

        Trajectory goToDepleteMedium1 = drive.trajectoryBuilder(forwardAndTurnMedium1.end().plus(new Pose2d(0, 0, forwardAndTurnMedium1TurnAngle)))
                        .forward(goToDepleteMedium)
                        .build();

        Trajectory awayFromJunctionMedium1 = drive.trajectoryBuilder(goToDepleteMedium1.end())
                        .back(awayFromJunctionMedium)
                        .build();


        waitForStart();
        signalSleeveNum = AprilTag.currentTagId(telemetry);
        telemetry.addData("tag", AprilTag.currentTagId(telemetry));
    }
}
