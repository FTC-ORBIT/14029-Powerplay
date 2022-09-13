import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

public class PoseTracker {
    private static Pose2D robotPose = new Pose2D(0, 0, 0);

    public static void update() {
        Drivetrain.drive.update();
        // * here all of the logic to decide if we can believe the sensors will go
        /*
         * if(distanceFromWall <= Constants.maxDistanceToBelieveDistanceSensor
         * && headingToBelieveDistanceSensorInTolerance){
         * x = cos(heading) * distanceFromWall
         * }
         * if(colorSensorInTolerance){
         * if(blue){
         * y = Constants.blueLineYPosition * Math.signum(robotPose.y)
         * }else{
         * y = Constants.redLineYPosition * Math.signum(robotPose.y)
         * }
         * }
         */

        robotPose = Drivetrain.drive.getPoseEstimate();
    }

    public static Pose2D getPosition() {
        return robotPose;
    }

    public static void setPosition(Pose2d pose2d) {
        Drivetrain.drive.setPoseEstimate(pose2d);
    }

}
