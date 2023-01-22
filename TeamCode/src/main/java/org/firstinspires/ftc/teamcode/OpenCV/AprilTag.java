package org.firstinspires.ftc.teamcode.OpenCV;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

public class AprilTag {

    public static ElapsedTime time = new ElapsedTime();

    public static OpenCvCamera camera;
    public static WebcamName webcamName;

    static final double FEET_PER_METER = 3.28084;
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;

    public static double tagsize = 0.166;

    public static AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(AprilTag.tagsize, AprilTag.fx, AprilTag.fy, AprilTag.cx, AprilTag.cy);


    // Tag ID 1,2,3 from the 36h11 family
    public static int LEFT = 0;
    public static int MIDDLE = 1;
    public static int RIGHT = 2;
    public static int retTag;
    public static boolean tagFound = false;

    public static AprilTagDetection tagOfInterest = null;

    public static void init(HardwareMap hardwareMap) {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        

    }


    public static void tagID(Telemetry telemetry) {

        telemetry.setMsTransmissionInterval(50);

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == RIGHT || tag.id == MIDDLE || tag.id == LEFT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    retTag = tagOfInterest.id;
                    break;
                }
            }

            if (tagFound) {

                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest, telemetry);
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest, telemetry);
                }
            }

        } else {
            telemetry.addLine("Don't see tag of interest :(");

            if (tagOfInterest == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest, telemetry);
            }

        }

        telemetry.update();
        time.reset();
        while(time.milliseconds() <= 20)



        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest, telemetry);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
//        if(tagOfInterest == null)
//        {
//            /*
//             * Insert your autonomous code here, presumably running some default configuration
//             * since the tag was never sighted during INIT
//             */
//        }
//        else
//        {
//            /*
//             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
//             */
//
//            // e.g.
//            if(tagOfInterest.pose.x <= 20)
//            {
//                // do something
//            }
//            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50)
//            {
//                // do something else
//            }
//            else if(tagOfInterest.pose.x >= 50)
//            {
//                // do something else
//            }
//        }
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            //trajectory... (lef)
        } else if (tagOfInterest.id == MIDDLE) {
            //trajectory... (middle)
        } else {
            //trajectory... (right)
        }
    }


    public static void tagToTelemetry(AprilTagDetection detection, Telemetry telemetry) {
        telemetry.addData("\nDetected tag ID=%d", detection.id);
        telemetry.addData("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER);
        telemetry.addData("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER);
        telemetry.addData("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER);
        telemetry.addData("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw));
        telemetry.addData("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch));
        telemetry.addData("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll));
    }

    public static int currentTagId(Telemetry telemetry) {
        tagID(telemetry);
        return retTag;
    }

}
