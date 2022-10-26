package org.firstinspires.ftc.teamcode.OrbitUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.positionTracker.PoseTracker;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class AutonomousFunction {
    public void updateAutonomousCode (RGB rgb){
        if (rgb.red > GlobalData.red){
            PoseTracker.setPose(new Pose2d()); // adjust it to the right side, the distance from the line to the further wall is 1.4605 meter, update the position according to the CS
        }
        if (rgb.blue > GlobalData.blue){
            PoseTracker.setPose(new Pose2d()); // adjust it to the right side, the distance from the line to the further wall is 1.4605 meter, update the position according to the CS
        }
    }
    //move this function as soon as there is gonna be an autonomous code
}
