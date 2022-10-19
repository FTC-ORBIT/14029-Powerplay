package org.firstinspires.ftc.teamcode.ImageProc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Config
@TeleOp(name = "Camera test")
public class OpenCv extends LinearOpMode {
    OpenCvCamera camera;
    Pipeline pipeline = new Pipeline(telemetry);
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                if(gamepad1.a) {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    camera.setPipeline(pipeline);
                    FtcDashboard.getInstance().startCameraStream(camera, 60);
                    TelemetryPacket packet = new TelemetryPacket();
                    //packet.put("place", pipeline.picCount);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                }
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }
    }



