package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.Sensors.DistSensor;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.positionTracker;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

@TeleOp(name = "main")
public class Robot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        PoseTracker.setPose(new Pose2d(0, 0, 0));

        waitForStart();
        while (!isStopRequested()) {
            GlobalData.currentTime = (float) time.seconds();
            Vector leftStick = new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
            Drivetrain.operate(leftStick, (float) PoseTracker.robotPose.heading);
            telemetry.update();
            PoseTracker.calcPose();

            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
            GlobalData.lastTime = GlobalData.currentTime;
        }
    }
}