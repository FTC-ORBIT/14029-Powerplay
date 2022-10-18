package org.firstinspires.ftc.teamcode.positionTracker;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;
import org.firstinspires.ftc.teamcode.Sensors.OrbitDistSensor;
import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

public class PoseTracker {
    private static Pose2d robotPose;
    private static Vector robotVelocity;
    private static Vector robotAcceleration;
    private static float omega;
    private static float lastAngle = (float) robotPose.getHeading();

    public static void calcPose() { // * we are calling this function every cycle in the opmode.
        Drivetrain.drive.update(); // ! this is the only place we should call the update function.
        robotPose = Drivetrain.getPose_FieldCS();
        calcPoseDerivatives();
    }

    public static void calcPoseDerivatives() {
        // TODO: here the velocity and acceleration will go.
        robotVelocity = getVelocity();
        robotAcceleration = getAcceleration();

    }

    public static Pose2d getPose() {
        return robotPose;
    }

    public static Vector getPosition() {
        return new Vector((float) robotPose.getX(), (float) robotPose.getY());
    }
    public static void initColor(HardwareMap hardwareMap){
        OrbitColorSensor.init(hardwareMap);
    }
    public static void updatePose(HardwareMap hardwareMap){
        String color = OrbitColorSensor.readColor(OrbitColorSensor.rgb()[0],OrbitColorSensor.rgb()[1],OrbitColorSensor.rgb()[2]);
        float distanceFromWall = OrbitDistSensor.getDistance();
        if (GlobalData.isAutonomous && distanceFromWall >= 30 && color == "red" || color == "blue"){
            // needs to be in double
            float[] xy = {(float) (Constants.distanceFromWall + (Constants.distanceOfColorFromMiddle * Math.abs(Math.cos(getHeading() + Constants.angleBetweenColorAndMiddle)))),
                    (float) (distanceFromWall * Math.abs(Math.cos(getHeading())))};
            if (color == "red"){
                if (GlobalData.blueOrRedStart_Red){
                    setPose(new Pose2d(xy[0],xy[1],getHeading()));
                }
                else {
                    setPose(new Pose2d(xy[0] + Constants.differenceInDisBetweenColors, xy[1], getHeading()));
                }
            }
            else if (color == "blue"){
                if (GlobalData.blueOrRedStart_Red){
                    setPose(new Pose2d(xy[0] + Constants.differenceInDisBetweenColors,xy[1], getHeading()));
                }
                else {
                    setPose(new Pose2d(xy[0], xy[1], getHeading()));
                }
            }
        }
        return;
    }
    public static float getHeading() {
        return (float) robotPose.getHeading();
    }

    public static float getOmega() {
        final float omega = (getHeading() - lastAngle) / GlobalData.deltaTime;
        lastAngle = getHeading();
        return omega;
    }

    public static Vector getVelocity() {
        return Drivetrain.getVelocity_FieldCS();
    }

    public static Vector getAcceleration() {
        return Drivetrain.getAcceleration();
    }

    public static float getSpeed() {
        return getVelocity().norm();
    }

    public static void setPose(Pose2d pose2d) {
        robotPose = pose2d;
        Drivetrain.drive.setPoseEstimate(robotPose);
    }

    public static void resetPosition() {
        setPose(new Pose2d(0, 0, 0));
    }


}
