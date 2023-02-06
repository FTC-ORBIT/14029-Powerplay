
package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class OrbitGyro {
    public static BNO055IMU imu;
    public static double  lastAngle = 0;
    static double currentAngle = 0;

    public static void init(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public static void resetGyro (){
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public static void resetGyroStartTeleop (float angle){
        if (angle < 290 && angle > 250) {
            lastAngle = -angle;
        } else if (!GlobalData.autonomousSide){
            lastAngle = -270;
        }
    }
    public static double getAngle() {
//        telemetry.addData("angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngle );
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngle;
    }

    public static double getDAngle (){
        double lastAngleT = currentAngle;
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngle;
        return currentAngle - lastAngleT;
    }
}
