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

    public static void run (boolean right, boolean first, boolean last, int tag){
        if (right){
            PoseTracker.setPose(new Pose2d((-3 * Constants.tileLength) + Constants.length * 0.5, -Constants.width * 0.5, 0 ));
        }else {
            PoseTracker.setPose(new Pose2d((-3 * Constants.tileLength) + Constants.length * 0.5, Constants.width * 0.5, 0 ));
        }
        if (first){
            Drivetrain.drive.followTrajectorySequence(firstToFidder(right));
            traj[1] = firstToFidder(right);
            //drive to (x, y * (right ? -1 : 1)
        } else {
            Drivetrain.drive.followTrajectorySequence(mainToFidder(right));
            traj[1] = mainToFidder(right);
        }

        Drivetrain.drive.followTrajectorySequence(cycles);

        switch (tag){
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

private static TrajectorySequence firstToFidder (boolean right){
        TrajectorySequence firstToFidder = Drivetrain.drive.trajectorySequenceBuilder(PoseTracker.getPose())
            .build();
        return firstToFidder;
}

private static TrajectorySequence mainToFidder (boolean right){
        TrajectorySequence mainToFidder = Drivetrain.drive.trajectorySequenceBuilder(PoseTracker.getPose())
            .build();
        return mainToFidder;
}

private static TrajectorySequence cycles = Drivetrain.drive.trajectorySequenceBuilder(traj[1].end())
            .build();
}

