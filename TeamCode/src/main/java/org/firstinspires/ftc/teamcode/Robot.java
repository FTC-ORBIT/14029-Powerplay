package org.firstinspires.ftc.teamcode;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.hardware.OrbitGyro;
import org.firstinspires.ftc.teamcode.positionTracker.PoseTracker;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;

@TeleOp(name = "main")
public class Robot extends OpMode {
    ElapsedTime time = new ElapsedTime();


    @Override
    public void init() {
        time.reset();
    }

    @Override
    public void loop() {

    }
}