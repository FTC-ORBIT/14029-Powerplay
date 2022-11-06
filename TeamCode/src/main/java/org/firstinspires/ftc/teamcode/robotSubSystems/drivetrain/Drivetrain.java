package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class Drivetrain {

    private static final DcMotor[] motors = new DcMotor[4];

    public static SampleMecanumDrive drive;

    private static Pose2d pose;
    static String motorWithProblem;
    static int timeWithInactiveMotor =0;
    static int[] motorsWithProblems ={0,0,0,0};
    static ElapsedTime time;
    public static Vector lastPosition;
    // equal to the last Autonomous position?
    public static Vector lastVelocity = GlobalData.inAutonomous ? getVelocity_FieldCS() : null;

    public static void init(HardwareMap hardwareMap) {
        if (GlobalData.inAutonomous) {
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive = new SampleMecanumDrive(hardwareMap);
        }
        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "rf");
        motors[2] = hardwareMap.get(DcMotor.class, "lb");
        motors[3] = hardwareMap.get(DcMotor.class, "rb");

        // TODO if your initial robot position is not 0,0,0 make sure to fix the
        // position (look for the function in the documentry). might be setPoseEstimate

        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public static void operate(final Vector velocity_W, float omega) {
        final float robotAngle = (float) Math.toRadians(OrbitGyro.getAngle());
        final Vector velocity_FieldCS_W = velocity_W.rotate(-robotAngle);
        drive(velocity_FieldCS_W, omega);
        if (GlobalData.inAutonomous) pose = drive.getPoseEstimate(); //// TODO: delete it because it may be useless
    }
    // did field centric

    public static Pose2d getPose_FieldCS() {
        return pose;
    }

    public static Vector getVelocity_FieldCS() {
        Vector position = new Vector((float) pose.getX(), (float) pose.getY());
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
    public static void tankDrive(Vector drive, double r, boolean RFandLB){
        double rfPower = 0;
        double lbPower = 0;
        double rbPower =0;
        double lfPower=0;

        if (RFandLB){
            rfPower = drive.y + r;
            lbPower = drive.y+r;
        }
        else{
            lfPower = drive.y + r;
            rbPower = drive.y - r;
        }
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
    /*public static void checkMotor(Vector drive, double r){
        //estimated speed
        /*double lfPower = drive.y + drive.x + r;
        double rfPower = drive.y - drive.x - r;
        double lbPower = drive.y - drive.x + r;
        double rbPower = drive.y + drive.x - r;
        double highestPower = 1;
        final double max = Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));
        if (max > 1)
            highestPower = max;

        lfPower = lfPower/highestPower;
        rfPower = rfPower/highestPower;
        lbPower = lbPower/highestPower;
        rbPower = rbPower/highestPower;

        if (Math.abs(lfPower)-Math.abs(motors[0].getPower()) < DrivetrainConstants.wheelErrorRange) {
            motorsWithProblems[0] = motorsWithProblems[0] + 1;
            time.reset();
        }
        if (Math.abs(rfPower)-Math.abs(motors[1].getPower()) < DrivetrainConstants.wheelErrorRange) {
            motorsWithProblems[1] = motorsWithProblems[1] + 1;
            time.reset();
        }
        if (Math.abs(lbPower)-Math.abs(motors[2].getPower()) < DrivetrainConstants.wheelErrorRange) {
            motorsWithProblems[2] = motorsWithProblems[2] + 1;
            time.reset();
        }
        if (Math.abs(rbPower)-Math.abs(motors[3].getPower()) < DrivetrainConstants.wheelErrorRange) {
            motorsWithProblems[3] = motorsWithProblems[3] + 1;
            time.reset();
        }

        if (timeWithInactiveMotor >= DrivetrainConstants.requiredTimeWithInactiveMotor){
            if (motorWithProblem == "rf" || motorWithProblem == "lb"){
                lfPower = drive.y + r;
                rbPower = drive.y - r;

                motors[0].setPower((lfPower));
                motors[1].setPower((0));
                motors[2].setPower((0));
                motors[3].setPower((rbPower));
            }
            else {

                rfPower = drive.y - r;
                lbPower = drive.y + r;
                motors[0].setPower((0));
                motors[1].setPower((rfPower));
                motors[2].setPower((lbPower));
                motors[3].setPower((0));
            }
        }
        else {
            if (time.milliseconds() < DrivetrainConstants.deltaTimeOfInactiveMotor)
                timeWithInactiveMotor++;
            else
                timeWithInactiveMotor = 0;
            if (motorsWithProblems[0] > motorsWithProblems[1] && motorsWithProblems[0] > motorsWithProblems[2] &&
                    motorsWithProblems[0] > motorsWithProblems[3])
                motorWithProblem = "lf";
            else if (motorsWithProblems[1] > motorsWithProblems[0] && motorsWithProblems[1] > motorsWithProblems[2] &&
                    motorsWithProblems[1] > motorsWithProblems[3])
                motorWithProblem = "rf";
            else if (motorsWithProblems[2] > motorsWithProblems[0] && motorsWithProblems[2] > motorsWithProblems[1] &&
                    motorsWithProblems[2] > motorsWithProblems[3])
                motorWithProblem = "lb";
            else
                motorWithProblem = "rb";
            motors[0].setPower((lfPower));
            motors[1].setPower((rfPower));
            motors[2].setPower((lbPower));
            motors[3].setPower((rbPower));

        }
         */
}
