package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.doge_cv.SkystoneDetector;
import org.firstinspires.ftc.teamcode.doge_cv.StoneDetector;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class basicAutoCustumade extends RobotCustomade {

    protected OpenCvCamera webcam;
    protected SkystoneDetector skystoneDetector;

    protected double Mikum = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

//        TODO: Webcam
/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        skystoneDetector = new SkystoneDetector(2,50,70);
        webcam.setPipeline(skystoneDetector);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
*/
//      TODO: Anothers
        Arm.setPosition(ArmClose);
        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        blinkinLedDriver.setPattern(pattern);

//        TODO: IMU

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStarted() && !isStopRequested() && !IMU.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addLine("IMU_IS_READY");
        telemetry.update();
    }
}
