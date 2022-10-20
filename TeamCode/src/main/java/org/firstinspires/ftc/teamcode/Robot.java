package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.positionTracker.PoseTracker;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

@TeleOp(name = "main")
public class Robot extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        time.reset();
        Drivetrain.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        PoseTracker.setPose(new Pose2d(0, 0, 0));

        waitForStart();
        while (!isStopRequested()) {
            GlobalData.currentTime = (float) time.seconds();
            Vector leftStick = new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
            Drivetrain.operate(leftStick, (float) PoseTracker.getHeading());
            telemetry.update();
            PoseTracker.calcPose(); //TODO: delete it because it may be useless

            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
            GlobalData.lastTime = GlobalData.currentTime;
        }
    }


}