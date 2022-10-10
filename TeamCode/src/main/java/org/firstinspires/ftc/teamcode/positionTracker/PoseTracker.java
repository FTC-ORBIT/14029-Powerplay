package org.firstinspires.ftc.teamcode.positionTracker;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Sensors.OrbitDistSensor;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;

public class PoseTracker {

    private static SampleMecanumDrive drive;
    private static Pose2d lastPosition = drive.getPoseEstimate(); // TODO is this equal to the last Autonomous position?
    private static ElapsedTime time = new ElapsedTime();
    private static Vector lastVelocity = getVelocity_FieldCS();
    private static float lastTime = 0;


    public static void init (HardwareMap hardwareMap){
        time.reset();
        drive = new SampleMecanumDrive(hardwareMap);
        // TODO if your initial robot position is not 0,0,0 make sure to fix the position (look for the function in the documentry). might be setPoseEstimate
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static Vector getVelocity_FieldCS() {
        drive.update();
        float currentTime = (float) time.seconds();
        Pose2d currentPosition = drive.getPoseEstimate();
        Pose2d deltaPose = currentPosition.minus(lastPosition); // TODO maybe a name a little more logical?
        float deltaTime = currentTime - lastTime;

        float xVelocity = (float) (deltaPose.getX() / deltaTime);
        float yVelocity = (float) (deltaPose.getY() / deltaTime);

        lastPosition = currentPosition;
        lastTime = currentTime;
        return new Vector(xVelocity, yVelocity);
    }

    public static float getSpeed() {
        return getVelocity_FieldCS().norm();
    }

    public static Pose2d getPose_FieldCS() {
     drive.update();
    return drive.getPoseEstimate();
    }


    public static Vector getAcceleration() {
        drive.update();
        float currentTime = (float) time.seconds();
        Vector currentVelocity = getVelocity_FieldCS();

        Vector deltaVelocity = currentVelocity.subtract(lastVelocity);
        float deltaTime = currentTime - lastTime;
        Vector acceleration = deltaVelocity.scale(1 / deltaTime);

        lastVelocity = currentVelocity;
        lastTime = currentTime;
        return acceleration;
    }

    public static void setPosition(Pose2d pose2d) {
      drive.setPoseEstimate(pose2d);
    }
    public static void resetPosition (){
        setPosition(new Pose2d(0,0,0));
    }

    public static Pose2d getPositionFromDistSensor() {
        double x = Math.sin(OrbitGyro.getAngle()) * OrbitDistSensor.getDistance();
        double y = Math.cos(OrbitGyro.getAngle()) * OrbitDistSensor.getDistance();
        Pose2d pose2d = new Pose2d().copy(x, y, OrbitGyro.getAngle());
        return pose2d;
    }

}
