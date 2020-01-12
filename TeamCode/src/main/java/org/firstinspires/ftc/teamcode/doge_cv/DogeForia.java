package org.firstinspires.ftc.teamcode.doge_cv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.PhotoID.VuforiaStone;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@TeleOp(name = "DogeForia", group = "DogeForia")
public class DogeForia extends LinearOpMode {

    protected VuforiaStone MyVuforiaStone = null;

    protected VuforiaLocalizer vuforia;
    protected OpenGLMatrix lastLocation = null;
    private static final float mmPerInch = 25.4f;
    public float Mikum = 0;
    public int vuforiastop = 0;

    protected static final String VUFORIA_KEY =
            "AShqm3D/////AAABmT32+8BbZEYfoY+L8BbhMAiFCWBAqEs1AghjDq2xOQw/uhnPZ4EVDEHOdbIxubuyTgO1mP2yAPzwlRyTTuBrTFIyVAUHjY0+j32GLbh0oLrKqnfyPtagrvZFS/YuAMhDNX25Uc1zXlD6iXX3pDoKBFuQLQ8zD/NvH5Ib2MTlMQq2srJpav6FRHGf0zU5OnEn1g+n2D5G3Uw7h19CyWFI/rQdUJ6kP2m9yMD8tAZDZiKhE0woZ/MgdGU5FgI6faiYCefYpLqrnW6ytWLenftxcKpccUHur1cWSjRxboVyPbVtgueWC7ytf0FrgyAvRo9uxGRXN6tYrjK1EZIPdssJ5PHxzWUd706EQvXIQwxd4Ndx";    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    protected VuforiaLocalizer.Parameters parametersVu;
    protected VuforiaTrackables targetsSkyStone;
    WebcamName webcamName = null;
    OpenCvWebcam webcam = null;
    OpenCvCamera cameraWebcam;

    SkystoneDetector skystoneDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("1");
        telemetry.update();
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVu = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        webcam = new OpenCvWebcam(webcamName, cameraMonitorViewId);

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        telemetry.addLine("2");
        telemetry.update();
        parametersVu.vuforiaLicenseKey = VUFORIA_KEY;
        parametersVu.cameraName = webcamName;

        telemetry.addLine("3");
        telemetry.update();
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parametersVu);
        //VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        MyVuforiaStone = new VuforiaStone( webcamName,parametersVu, targetsSkyStone, vuforia,lastLocation);

        telemetry.addLine("4");
        telemetry.update();

//        webcam = new OpenCvWebcam(webcamName, cameraMonitorViewId);

        telemetry.addLine("5");
        telemetry.update();

        skystoneDetector = new SkystoneDetector();
        webcam.setPipeline(skystoneDetector);

        webcam.openCameraDevice();
//        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);


        while (!isStarted()){
            telemetry.addLine("here");
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()){
//            Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();
            telemetry.addLine("Telemetry");
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }
}
