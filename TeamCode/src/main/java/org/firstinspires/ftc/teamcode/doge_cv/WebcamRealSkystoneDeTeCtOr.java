package org.firstinspires.ftc.teamcode.doge_cv;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.PhotoID.VuforiaStone;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * Thanks to EasyOpenCV for the great API (and most of the example)
 *
 * Original Work Copright(c) 2019 OpenFTC Team
 * Derived Work Copyright(c) 2019 DogeDevs
 */
@TeleOp(name = "WebcamRealSkystoneDeTeCtOr", group="DogeCV")

public class WebcamRealSkystoneDeTeCtOr extends LinearOpMode {
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
    SuperWebcamName superWebcamName = null;

    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */

        superWebcamName = new SuperWebcamName(hardwareMap.get(OpenCvWebcam.class, "Webcam 1"),
                                                hardwareMap.get(WebcamName.class, "Webcam 1"));

                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVu = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parametersVu.vuforiaLicenseKey = VUFORIA_KEY;
        parametersVu.cameraName = superWebcamName.webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parametersVu);
        //VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        MyVuforiaStone = new VuforiaStone(superWebcamName.webcamName, parametersVu, targetsSkyStone, vuforia,lastLocation);


/*        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new WebcamNameMerged(webcam);
        webcam = hardwareMap.get(WebcamNameMerged.class, "Webcam 1");
        webcam.openCameraDevice();
        skyStoneDetector = new SkystoneDetector();
        webcam.setPipeline(skyStoneDetector);*/

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
//        TODO: change tesolute
        superWebcamName.OpenCv.startStreamingImplSpecific(320, 240);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Mikum:", Mikum);
            Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                superWebcamName.openCvWebcam.closeCameraDevice();
            }

            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * The "if" statements below will pause the viewport if the "X" button on gamepad1 is pressed,
             * and resume the viewport if the "Y" button on gamepad1 is pressed.
             */
            else if(gamepad1.x) {
                superWebcamName.openCvWebcam.pauseViewport();
            }
            else if(gamepad1.y) {
                superWebcamName.openCvWebcam.resumeViewport();
            }
        }
    }
}