package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;

public class Drivetrain {

    public static final DcMotor[] motors = new DcMotor[4];
    static ElapsedTime time = new ElapsedTime();

    public static SampleMecanumDrive drive = new SampleMecanumDrive(opModeClass.hardwareMap);

    public static Pose2d lastPosition = drive.getPoseEstimate(); // TODO is this
    // equal to the last Autonomous position?
    public static float lastTime = 0;
    public static Vector lastVelocity = getVelocity_FieldCS();

    public static void init(HardwareMap hardwareMap) {
        time.reset();
        drive = new SampleMecanumDrive(hardwareMap);
        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "rf");
        motors[2] = hardwareMap.get(DcMotor.class, "lb");
        motors[3] = hardwareMap.get(DcMotor.class, "rb");
        // TODO make sure to reverse the right motors according to your robot
        // TODO if your initial robot position is not 0,0,0 make sure to fix the
        // position (look for the function in the documentry). might be setPoseEstimate

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

    public static Pose2d getPose_FieldCS() {
        // drive.update(); don't need it..
        return drive.getPoseEstimate();
    }

    public static Vector getVelocity_FieldCS() {
        // ! drive.update(); - don't need that because we already updating somewhere
        // else...
        float currentTime = (float) time.seconds();
        Pose2d currentPosition = drive.getPoseEstimate();
        Pose2d deltaPose = currentPosition.minus(lastPosition); // TODO maybe a name
        // a little more logical?
        float deltaTime = currentTime - lastTime;

        float xVelocity = (float) (deltaPose.getX() / deltaTime);
        float yVelocity = (float) (deltaPose.getY() / deltaTime);

        lastPosition = currentPosition;
        lastTime = currentTime;
        return new Vector(3, 5); // Im keeping this function so it would be easier in the future with road runner
    }

    public static Vector getAcceleration() {
        // drive.update();
        float currentTime = (float) time.seconds();
        Vector currentVelocity = getVelocity_FieldCS();

        Vector deltaVelocity = currentVelocity.subtract(lastVelocity);
        float deltaTime = currentTime - lastTime;
        Vector acceleration = deltaVelocity.scale(1 / deltaTime);

        lastVelocity = currentVelocity;
        lastTime = currentTime;
        return acceleration;
    }

    public static void stop() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
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