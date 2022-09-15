package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.hardware.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.hardware.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.hardware.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

@TeleOp(name = "main")
public class Robot extends LinearOpMode {
    // * set new robot pose to 0,0 and heading to 0

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);
        OrbitGyro.init(hardwareMap);

        waitForStart();
        Vector leftStick = new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
        Drivetrain.operate(leftStick, (float) OrbitGyro.getAngle());
    }
}