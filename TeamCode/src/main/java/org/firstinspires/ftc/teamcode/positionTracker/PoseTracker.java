package org.firstinspires.ftc.teamcode.positionTracker;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

public class PoseTracker {
    private static Pose2d robotPose;

    public static void calcPose() { // * we are calling this function every cycle in the opmode.
        Drivetrain.drive.update(); // ! this is the only place we should call the update function.
        robotPose = Drivetrain.getPose_FieldCS();
        calcPoseDerivatives();
    }

    public static void calcPoseDerivatives() {
        // TODO: here the velocity and acceleration will go.
    }

    public static Pose2d getPose() {
        return robotPose;
    }

    public static void setPose(Pose2d pose2d) {
        robotPose = pose2d;
        Drivetrain.drive.setPoseEstimate(robotPose);
    }

    public static void resetPosition() {
        setPose(new Pose2d(0, 0, 0));
    }
}
