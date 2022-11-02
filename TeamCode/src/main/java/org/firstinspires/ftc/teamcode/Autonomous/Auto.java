package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.OrbitUtils.RGB;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.positionTracker.PoseTracker;
import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;

public class Auto {

    private static TrajectorySequence[] traj;

    public static void run(boolean right, boolean first, boolean last, int tag) {

        if (right) {
            PoseTracker.setPose(new Pose2d((-3 * Constants.tileLength) + Constants.robotLength * 0.5, -((Constants.robotWidth * 0.5) + Constants.tileLength), 0));
        } else {
            PoseTracker.setPose(new Pose2d((-3 * Constants.tileLength) + Constants.robotLength * 0.5, (Constants.robotWidth * 0.5) + Constants.tileLength, 0));
        }
        PoseTracker.calcPose();
        TrajectorySequence elevator = Drivetrain.drive.trajectorySequenceBuilder(PoseTracker.getPose())
                .addDisplacementMarker(() -> {
                    Elevator.operate(ElevatorStates.HIGH, null);
                })
                .build();
        if (first) {
            TrajectorySequence firstToMainJunction = Drivetrain.drive.trajectorySequenceBuilder(PoseTracker.getPose())
                    .strafeTo(new Vector2d( PoseTracker.getPose().getX(), Constants.tileLength * (right ? -0.5 : 0.5)))
                    .lineToLinearHeading(new Pose2d(-Constants.tileLength, Constants.tileLength * (right ? -0.5 : 0.5), Math.toRadians(90 * (right ? 1 : -1))))
                    .addDisplacementMarker(() -> {
                        Claw.operate(ClawState.CLOSE);
                    })
                    .build();
            TrajectorySequence mainJunctionToFidder = Drivetrain.drive.trajectorySequenceBuilder(firstToMainJunction.end())
                    .strafeTo(new Vector2d(-0.5 * Constants.tileLength, Constants.tileLength * (right ? -0.5 : 0.5)))
                    .lineToLinearHeading(new Pose2d(-0.5 * Constants.tileLength, Constants.tileLength * (right? -1 : 1), Math.toRadians(90) * (right? -1 : 1 )))
                                    .build();

            Drivetrain.drive.followTrajectorySequence(firstToMainJunction);
            while (GlobalData.hasGamePiece){}
            Drivetrain.drive.followTrajectorySequence(mainJunctionToFidder);

            traj[0] = mainJunctionToFidder;
            //drive to (x, y * (right ? -1 : 1)
        } else {
            TrajectorySequence mainToFidder = Drivetrain.drive.trajectorySequenceBuilder(PoseTracker.getPose())
                    .build();
            Drivetrain.drive.followTrajectorySequence(mainToFidder);
            traj[0] = mainToFidder;
        }

        TrajectorySequence cycles = Drivetrain.drive.trajectorySequenceBuilder(traj[0].end())
                .build();                                               // TODO I must define cycles here because only here taj [0] is updated to the current trajectory

        Drivetrain.drive.followTrajectorySequence(cycles);

        switch (tag) {
            case 1:
                TrajectorySequence firstPark = Drivetrain.drive.trajectorySequenceBuilder(cycles.end())
                        .build();
                Drivetrain.drive.followTrajectorySequence(firstPark);
                break;
            case 2:
                TrajectorySequence secondPark = Drivetrain.drive.trajectorySequenceBuilder(cycles.end())
                        .build();
                Drivetrain.drive.followTrajectorySequence(secondPark);
                break;
            case 3:
                TrajectorySequence thirdPark = Drivetrain.drive.trajectorySequenceBuilder(cycles.end())
                        .build();
                Drivetrain.drive.followTrajectorySequence(thirdPark);
                break;
        }

    }
}
