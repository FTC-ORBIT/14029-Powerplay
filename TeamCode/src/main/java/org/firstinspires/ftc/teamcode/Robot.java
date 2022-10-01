package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.DistSensor;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.positionTracker;

@TeleOp(name = "main")
public class Robot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        PoseTracker.setPose(new Pose2d(0, 0, 0));

        waitForStart();
        while (!isStopRequested()) {
            Vector leftStick = new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
            Drivetrain.operate(leftStick, (float) OrbitGyro.getAngle());
            telemetry.update();
            PoseTracker.calcPose();
        }
    }
}