package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OrbitUtils.RGB;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.positionTracker.PoseTracker;
import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

public class Auto {

    private static TrajectorySequence[] traj;

    public static void run(boolean right, boolean first, boolean last, int tag) {

        //put here the image processing code for the randomization

        if (right) {
            PoseTracker.setPose(new Pose2d((-3 * Constants.tileLength) + Constants.robotLength * 0.5, -((Constants.robotWidth * 0.5) + Constants.tileLength) , 0));
        } else {
            PoseTracker.setPose(new Pose2d((-3 * Constants.tileLength) + Constants.robotLength * 0.5, (Constants.robotWidth * 0.5) + Constants.tileLength, 0));
        }
        PoseTracker.calcPose();

        if (first) {
            TrajectorySequence firstToFidder = Drivetrain.drive.trajectorySequenceBuilder(PoseTracker.getPose())
                        .build();
            Drivetrain.drive.followTrajectorySequence(firstToFidder);
            traj[0] = firstToFidder;
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

