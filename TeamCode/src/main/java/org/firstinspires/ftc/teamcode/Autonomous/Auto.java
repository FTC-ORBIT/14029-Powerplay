package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.RGB;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitDistanceSensor;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.positionTracker.PoseTracker;
import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorStates;

public class Auto {

    public static OrbitColorSensor colorSensor;
    public static OrbitDistanceSensor distanceSensor;
    private static TrajectorySequence[] traj;
    private static int cyclesWantedNum = 3;
    private static int cyclesCurrentNum;

    public static void init(HardwareMap hardwareMap) {
        colorSensor = new OrbitColorSensor(hardwareMap);
        distanceSensor = new OrbitDistanceSensor(hardwareMap);
        cyclesWantedNum = 3;
        cyclesCurrentNum = 0;
    }

    public static void run(boolean right, boolean first, boolean last, int tag) {

        if (right) {
            PoseTracker.setPose(new Pose2d((-3 * Constants.tileLength) + Constants.robotLength * 0.5, -((Constants.robotWidth * 0.5) + Constants.tileLength), 0));
        } else {
            PoseTracker.setPose(new Pose2d((-3 * Constants.tileLength) + Constants.robotLength * 0.5, (Constants.robotWidth * 0.5) + Constants.tileLength, 0));
        }
        PoseTracker.getPose();
        TrajectorySequence elevator = Drivetrain.drive.trajectorySequenceBuilder(PoseTracker.getPose())
                .addDisplacementMarker(() -> {
                    Elevator.operate(ElevatorStates.HIGH, null);
                })
                .build();
        if (first) {
            TrajectorySequence firstToInitialJunction = Drivetrain.drive.trajectorySequenceBuilder(PoseTracker.getPose())
                    .addDisplacementMarker(() -> {
                        Claw.operate(ClawState.OPEN);
                    })
                    .build();
            TrajectorySequence initialJunctionToFidder = Drivetrain.drive.trajectorySequenceBuilder(firstToInitialJunction.end())
                    .addDisplacementMarker(() -> {
                        Claw.operate(ClawState.CLOSE);
                    })
                    .build();

            Drivetrain.drive.followTrajectorySequence(firstToInitialJunction);
            while (GlobalData.hasGamePiece) {
            }
            Drivetrain.drive.followTrajectorySequence(initialJunctionToFidder);
            while (!Claw.isClawCorrectPos(ClawConstants.closed)) {
            }

            traj[0] = initialJunctionToFidder;
            //drive to (x, y * (right ? -1 : 1)
        } else {
            TrajectorySequence firstToMainJunction = Drivetrain.drive.trajectorySequenceBuilder(PoseTracker.getPose())
                    .addDisplacementMarker(() -> {
                        Claw.operate(ClawState.OPEN);
                    })
                    .build();
            TrajectorySequence mainJunctionToFidder = Drivetrain.drive.trajectorySequenceBuilder(firstToMainJunction.end())
                    .addDisplacementMarker(() -> {
                        Claw.operate(ClawState.CLOSE);
                    })
                    .build();

            Drivetrain.drive.followTrajectorySequence(firstToMainJunction);
            while (GlobalData.hasGamePiece) {
            }
            Drivetrain.drive.followTrajectorySequence(mainJunctionToFidder);
            while (!Claw.isClawCorrectPos(ClawConstants.closed)) {
            }

            traj[0] = mainJunctionToFidder;
        }


        TrajectorySequence depleteCycle = Drivetrain.drive.trajectorySequenceBuilder(traj[0].end())
                .build();
        TrajectorySequence intakeCycle = Drivetrain.drive.trajectorySequenceBuilder(depleteCycle.end())
                .build(); // TODO I must define cycles here because only here taj [0] is updated to the current trajectory


        while (cyclesCurrentNum <= cyclesWantedNum) {
            Drivetrain.drive.followTrajectorySequence(depleteCycle);
            while (GlobalData.hasGamePiece) {}
            Drivetrain.drive.followTrajectorySequence(intakeCycle);
            while (!Claw.isClawCorrectPos(ClawConstants.closed)) {}
            cyclesCurrentNum++;
        }

        if (last){
            TrajectorySequence fidderToLast = Drivetrain.drive.trajectorySequenceBuilder(intakeCycle.end())
                    .build();
            Drivetrain.drive.followTrajectorySequence(fidderToLast);
        } else {
            Drivetrain.drive.followTrajectorySequence(depleteCycle);
        }
        while (GlobalData.hasGamePiece) {}

        //TODO do we need to write a different code for parking in case last == true?

        switch (tag) {
            case 1:
                TrajectorySequence firstPark = Drivetrain.drive.trajectorySequenceBuilder(depleteCycle.end())
                        .build();
                Drivetrain.drive.followTrajectorySequence(firstPark);
                break;
            case 2:
                TrajectorySequence secondPark = Drivetrain.drive.trajectorySequenceBuilder(depleteCycle.end())
                        .build();
                Drivetrain.drive.followTrajectorySequence(secondPark);
                break;
            case 3:
                TrajectorySequence thirdPark = Drivetrain.drive.trajectorySequenceBuilder(depleteCycle.end())
                        .build();
                Drivetrain.drive.followTrajectorySequence(thirdPark);
                break;
        }

    }
}
