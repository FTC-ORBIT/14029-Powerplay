package org.firstinspires.ftc.teamcode.hardware.RoadRunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

       // Trajectory traj = drive.trajectoryBuilder(new Pose2d())
       //         .splineTo(new Vector2d(1, 1), 0)
       //         .build();

       // drive.followTrajectory(traj);

       // sleep(2000);

       // drive.followTrajectory(
       //         drive.trajectoryBuilder(traj.end(), true)
       //                 .splineTo(new Vector2d(0, 0), Math.toRadians(180))
        //                 .build()
        // );

        Trajectory cycle1 = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(1.28,0.176, Math.toRadians(54)), 90).build();
        Trajectory backToTheWarHouse1 = drive.trajectoryBuilder(cycle1.end(), true)
                .splineToLinearHeading(new Pose2d(0, 0, 0), 90)
                .build();
        Trajectory cycle2 = drive.trajectoryBuilder(backToTheWarHouse1.end())
                .splineToLinearHeading(new Pose2d(1.28,0.176, Math.toRadians(54)), 90).build();
        Trajectory backToTheWarHouse2 = drive.trajectoryBuilder(cycle2.end(), true)
                .splineToLinearHeading(new Pose2d(0, 0, 0), 90)
                .build();
        Trajectory cycle3 = drive.trajectoryBuilder(backToTheWarHouse2.end())
                .splineToLinearHeading(new Pose2d(1.28,0.176, Math.toRadians(54)), 90).build();
        Trajectory backToTheWarHouse3 = drive.trajectoryBuilder(cycle3.end(), true)
                .splineToLinearHeading(new Pose2d(0, 0, 0), 90)
                .build();
        Trajectory cycle4 = drive.trajectoryBuilder(backToTheWarHouse3.end())
                .splineToLinearHeading(new Pose2d(1.28,0.176, Math.toRadians(54)), 90).build();
        Trajectory backToTheWarHouse4 = drive.trajectoryBuilder(cycle4.end(), true)
                .splineToLinearHeading(new Pose2d(0, 0, 0), 90)
                .build();
        Trajectory cycle5 = drive.trajectoryBuilder(backToTheWarHouse4.end())
                .splineToLinearHeading(new Pose2d(1.28,0.176, Math.toRadians(54)), 90).build();
        Trajectory backToTheWarHouse5 = drive.trajectoryBuilder(cycle5.end(), true)
                .splineToLinearHeading(new Pose2d(0, 0, 0), 90)
                .build();
        Trajectory cycle6 = drive.trajectoryBuilder(backToTheWarHouse5.end())
                .splineToLinearHeading(new Pose2d(1.28,0.176, Math.toRadians(54)), 90).build();
        Trajectory backToTheWarHouse6 = drive.trajectoryBuilder(cycle6.end(), true)
                .splineToLinearHeading(new Pose2d(0, 0, 0), 90)
                .build();
        drive.followTrajectory(cycle1);
        drive.followTrajectory(backToTheWarHouse1);
        drive.followTrajectory(cycle2);
        drive.followTrajectory(backToTheWarHouse2);
        drive.followTrajectory(cycle3);
        drive.followTrajectory(backToTheWarHouse3);
        drive.followTrajectory(cycle4);
        drive.followTrajectory(backToTheWarHouse4);
        drive.followTrajectory(cycle5);
        drive.followTrajectory(backToTheWarHouse5);
        drive.followTrajectory(cycle6);
        drive.followTrajectory(backToTheWarHouse6);
    }
}
