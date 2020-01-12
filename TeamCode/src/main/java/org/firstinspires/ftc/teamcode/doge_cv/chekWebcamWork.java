package org.firstinspires.ftc.teamcode.doge_cv;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.basicAuto;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;


/*
 * Thanks to EasyOpenCV for the great API (and most of the example)
 *
 * Original Work Copright(c) 2019 OpenFTC Team
 * Derived Work Copyright(c) 2019 DogeDevs
 */
@Disabled
@TeleOp(name = "chekWebcamWork", group = "DogeCV")
public class chekWebcamWork extends basicAuto {

    private WebcamName webcamName = null;
    private OpenCvWebcam webcam;
    private SkystoneDetector skyStoneDetector;
    @Override
    public void runOpMode() {


        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); //Retrieves the webcam from the hardware map

        skyStoneDetector = new SkystoneDetector(); // Create a Gold Detector

        //Sets the Vuforia license key. ALWAYS SET BEFORE INIT!

        //Inits the detector. Choose which camera to use, and whether to detect VuMarks here
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(webcamName,cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.openCameraDeviceImplSpecific();
        webcam.setPipeline(skyStoneDetector);
        webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
            telemetry.addData("Webcam FPS", webcam.getFps());
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }

}