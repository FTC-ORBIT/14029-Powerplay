package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.positionTracker.Pose2DP;

public class Drivetrain {

    public static final DcMotor[] motors = new DcMotor[4];
    static ElapsedTime time = new ElapsedTime();
    public static SampleMecanumDrive drive ;//= new SampleMecanumDrive(opModeClass.hardwareMap);
    public static Pose2d lastPosition = drive.getPoseEstimate(); //TODO is this equal to the last Autonomous position?
    public static float lastTime = 0;
    public static Vector lastVelocity = getVelocity_FieldCS();

    public static void init(HardwareMap hardwareMap) {
        time.reset();
        drive = new SampleMecanumDrive(hardwareMap);
        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "rf");
        motors[2] = hardwareMap.get(DcMotor.class, "lb");
        motors[3] = hardwareMap.get(DcMotor.class, "rb");
        motors [0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors [2].setDirection(DcMotorSimple.Direction.REVERSE);
        //TODO make sure to reverse the right motors according to your robot
        //TODO if your initial robot position is not 0,0,0 make sure to fix the position (look for the function in the documentry). might be setPoseEstimate
        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public static void operate(final Vector velocity_W, float omega) {
        final float robotAngle = (float) Math.toRadians(OrbitGyro.getAngle());
        final Vector velocity_FieldCS_W = velocity_W.rotate(-robotAngle);
        drive(velocity_FieldCS_W, omega);
    }
    // did field centric

    public static Vector getVelocity_FieldCS (){
        drive.update();
        float currentTime = (float) time.seconds();
        Pose2d currentPosition = drive.getPoseEstimate();
        Pose2d deltaPose = currentPosition.minus(lastPosition); //TODO maybe a name a little more logical?
        float deltaTime = currentTime - lastTime;

        float xVelocity = (float) (deltaPose.getX() / deltaTime);
        float yVelocity = (float) (deltaPose.getY() / deltaTime);

        lastPosition = currentPosition;
        lastTime = currentTime;
        return new Vector(xVelocity, yVelocity);
    }
    public static float getSpeed (){
        return getVelocity_FieldCS().norm();
    }
    public static Pose2d getPose_FieldCS (){
        drive.update();
        return drive.getPoseEstimate();
    }

    public static Vector getAcceleration (){
        drive.update();
        float currentTime = (float) time.seconds();
        Vector currentVelocity = getVelocity_FieldCS();

        Vector deltaVelocity = currentVelocity.subtract(lastVelocity);
        float deltaTime = currentTime - lastTime;
        Vector acceleration = deltaVelocity.scale(1/deltaTime);

        lastVelocity = currentVelocity;
        lastTime = currentTime;
        return acceleration;
    }

    public static void stop() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }
    public static void setPosition(Pose2d pose2d){
    drive.setPoseEstimate(pose2d);
    }


    public static void drive(Vector drive, double r) {
        final double lfPower = drive.y + drive.x + r;
        final double rfPower = drive.y - drive.x - r;
        final double lbPower = drive.y - drive.x + r;
        final double rbPower = drive.y + drive.x - r;
        double highestPower = 1;
        final double max = Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));
        if (max > 1)
            highestPower = max;
        motors[0].setPower((lfPower / highestPower));
        motors[1].setPower((rfPower / highestPower));
        motors[2].setPower((lbPower / highestPower));
        motors[3].setPower((rbPower / highestPower));
    }

}
