package org.firstinspires.ftc.teamcode.doge_cv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotCustomade;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "StoneDetectorWebcamTest", group = "dogeCv")
public class StoneDetectorWebcamTest extends LinearOpMode {

    protected OpenCvCamera webcam;
    protected StoneDetector stonedetector;

    @Override
    public void runOpMode() throws InterruptedException {

//        TODO: Webcam

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        stonedetector = new StoneDetector();
        webcam.setPipeline(stonedetector);

        webcam.openCameraDevice();

        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        while (!isStarted()){
            if (stonedetector.isDetected()) {
                telemetry.addData("size", stonedetector.foundRectangles().size());
                try {
                    for (int i = 0; i < stonedetector.foundRectangles().size(); i++) {
                        telemetry.addData("Stone" + (i + 1), stonedetector.foundRectangles().get(i).toString());
                    }
                } catch (Exception e) {
                    telemetry.addData("Stones", "Not Detected");
                }
            }
            telemetry.addData("Height", stonedetector.getSize().height);
            telemetry.addData("width", stonedetector.getSize().width);
            telemetry.addData("stonesToFind", stonedetector.stonesToFind);
            telemetry.addData("stonesToFind", stonedetector.foundScreenPositions().get(30));
            telemetry.update();
        }
    }
}
