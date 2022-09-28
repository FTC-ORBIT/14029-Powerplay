package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sensors.DistSensor;

@TeleOp(name = "test")
public class distSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DistSensor.init(hardwareMap);

        waitForStart();

        telemetry.addData("sensorRead", DistSensor.readDistance());
        telemetry.update();
    }
}
