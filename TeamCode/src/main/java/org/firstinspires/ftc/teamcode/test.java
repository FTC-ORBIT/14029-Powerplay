package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawConstants;

@TeleOp(name = "test")
@Config
public class test extends LinearOpMode {

    public static double pos = ClawConstants.open;


    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        Claw.init(hardwareMap);


        waitForStart();

        while (!isStopRequested()){

            Claw.test(pos);

            packet.put("pos", Claw.clawServo.getPosition());
            dashboard.sendTelemetryPacket(packet);

        }
    }
}
