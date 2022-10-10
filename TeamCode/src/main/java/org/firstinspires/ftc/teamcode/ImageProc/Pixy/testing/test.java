package org.firstinspires.ftc.teamcode.ImageProc.Pixy.testing;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ImageProc.Pixy.hardware.PixyBlock;
import org.firstinspires.ftc.teamcode.ImageProc.Pixy.hardware.PixyBlockList;
import org.firstinspires.ftc.teamcode.ImageProc.Pixy.hardware.PixyCam;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

/**
 * Source code brought in from:
 * https://github.com/Overlake-FTC-7330-2017/ftc_app/blob/TeamCode/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/testing/TestPixyCam.java
 * on 2017-10-19
 */

@TeleOp(name = "TestPixyCam")
public class test extends OpMode {
    PixyCam pixyCam;
    PixyBlockList blocks1;
    PixyBlockList blocks2;
    PixyBlockList blocks3;
    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime elapsedTime2 = new ElapsedTime();

    PrintWriter file;

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        pixyCam = hardwareMap.get(PixyCam.class, "pixycam");
        try {
            file = new PrintWriter("/sdcard/pixyResults.txt", "UTF-8");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        // Update every tenth of a second.
        if (elapsedTime.milliseconds() > 100) {
            elapsedTime.reset();
            blocks1 = pixyCam.getBiggestBlocks(1);
            blocks2 = pixyCam.getBiggestBlocks(2);
            blocks3 = pixyCam.getBiggestBlocks(3);
            telemetry.addData("Counts", "%d/%d/%d", blocks1.totalCount, blocks2.totalCount, blocks3.totalCount);
            file.println("----------------------------");
            file.format("Elapsed: %s Counts: %d/%d\n", elapsedTime2.toString(), blocks2.totalCount, blocks3.totalCount);
            for (int i = 0; i < blocks1.size(); i++) {
                PixyBlock block = blocks1.get(i);
                if (!block.isEmpty()) {
                    telemetry.addData("Block 1[" + i + "]", block.toString());
                }
            }
            for (int i = 0; i < blocks2.size(); i++) {
                PixyBlock block = blocks2.get(i);
                if (!block.isEmpty()) {
                    telemetry.addData("Block 2[" + i + "]", block.toString());
                    file.format("Block2[%d]: %s\n", i, block.toString());
                }
            }
            for (int i = 0; i < blocks3.size(); i++) {
                PixyBlock block = blocks3.get(i);
                if (!block.isEmpty()) {
                    telemetry.addData("Block 3[" + i + "]", block.toString());
                    file.format("Block3[%d]: %s\n", i, block.toString());
                }
            }
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        file.close();
    }

}