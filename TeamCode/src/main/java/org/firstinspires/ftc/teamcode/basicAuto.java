package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class basicAuto extends Robot {

    protected VuforiaStone MyVuforiaStone = null;

    protected VuforiaLocalizer vuforia;
    protected OpenGLMatrix lastLocation = null;
    private static final float mmPerInch = 25.4f;
    public float Mikum=0;
    public int vuforiastop = 0;

    protected static final String VUFORIA_KEY =
            "AShqm3D/////AAABmT32+8BbZEYfoY+L8BbhMAiFCWBAqEs1AghjDq2xOQw/uhnPZ4EVDEHOdbIxubuyTgO1mP2yAPzwlRyTTuBrTFIyVAUHjY0+j32GLbh0oLrKqnfyPtagrvZFS/YuAMhDNX25Uc1zXlD6iXX3pDoKBFuQLQ8zD/NvH5Ib2MTlMQq2srJpav6FRHGf0zU5OnEn1g+n2D5G3Uw7h19CyWFI/rQdUJ6kP2m9yMD8tAZDZiKhE0woZ/MgdGU5FgI6faiYCefYpLqrnW6ytWLenftxcKpccUHur1cWSjRxboVyPbVtgueWC7ytf0FrgyAvRo9uxGRXN6tYrjK1EZIPdssJ5PHxzWUd706EQvXIQwxd4Ndx";    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    protected VuforiaLocalizer.Parameters parametersVu;
    protected VuforiaTrackables targetsSkyStone;
    WebcamName webcamName = null;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVu = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parametersVu.vuforiaLicenseKey = VUFORIA_KEY;
        parametersVu.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parametersVu);
        //VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        MyVuforiaStone = new VuforiaStone( webcamName,parametersVu, targetsSkyStone, vuforia,lastLocation);
    }
}
