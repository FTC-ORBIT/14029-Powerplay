package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class Drivetrain {

    private static final DcMotor[] motors = new DcMotor[4];

    public static SampleMecanumDrive drive;

    private static Pose2d pose;
    public static Vector lastPosition;
    // equal to the last Autonomous position?
    public static float lastTime = 0;
    public static Vector lastVelocity = GlobalData.inAutonomous == true ? getVelocity_FieldCS() : null;

    public static void init(HardwareMap hardwareMap) {
        if (GlobalData.inAutonomous) drive = new SampleMecanumDrive(hardwareMap);
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
        pose = drive.getPoseEstimate(); ////TODO: delete it because it may be useless
    }
    // did field centric

    public static Pose2d getPose_FieldCS() {
        return pose;
    }

    public static Vector getVelocity_FieldCS() {
        Vector position = new Vector((float)pose.getX(),(float) pose.getY());
        Vector deltaPosition = position.subtract(lastPosition);

        final Vector velocity = deltaPosition.scale(1 / GlobalData.deltaTime);

        lastPosition = position;
        return velocity;
    }

    public static Vector getAcceleration() {
        Vector currentVelocity = getVelocity_FieldCS();

        Vector deltaVelocity = currentVelocity.subtract(lastVelocity);
        Vector acceleration = deltaVelocity.scale(1 / GlobalData.deltaTime);

        lastVelocity = currentVelocity;
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