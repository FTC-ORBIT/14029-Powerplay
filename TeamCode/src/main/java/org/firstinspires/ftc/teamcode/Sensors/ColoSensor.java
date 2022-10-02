package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(group =  "ColorSensor")
public class ColoSensor extends LinearOpMode {


    ColorSensor color;
    @Override
    public void runOpMode() throws InterruptedException {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (!isStopRequested()) {
            telemetry.addData("Red", ((255 * color.red())/4095) > 255 ? 255 : ((255 * color.red())/4095));
            telemetry.addData("Green", ((255 * color.green())/4095) > 255 ? 255 : ((255 * color.green())/4095));
            telemetry.addData("Blue", ((255 * color.blue())/4095) > 255 ? 255 : ((255 * color.blue())/4095));
            telemetry.update();
        }
    }
    }

