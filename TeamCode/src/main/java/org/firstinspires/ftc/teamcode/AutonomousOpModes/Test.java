package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.OrbitUtils.PID;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

@Config
@Autonomous(group = "Test")
public class Test extends LinearOpMode {
    public static double kpForward = 0.7;
    PID forward = new PID(kpForward, 0, 0, 0, 0);
    public static double kpAngle = 0;
    PID angle = new PID(kpAngle, 0, 0, 0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        forward.setWanted(1.5);
        angle.setWanted(0);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()){
            float distance = (float) ((Drivetrain.motors[2].getCurrentPosition() / 8192) * 0.0175 * 2 * Math.PI);
            double yPower = forward.update(distance);
            double anglePower = angle.update(OrbitGyro.getAngle());
            Drivetrain.drive(new Vector(0, (float) yPower), anglePower);
            telemetry.update();
            telemetry.addData("distance", distance);
            telemetry.addData("encoder", Drivetrain.motors[2].getCurrentPosition());
            telemetry.addData("yPower", yPower);
            telemetry.addData("angle", anglePower);
        }
    }
}
