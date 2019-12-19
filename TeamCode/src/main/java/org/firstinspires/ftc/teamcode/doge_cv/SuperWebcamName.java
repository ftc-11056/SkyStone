package org.firstinspires.ftc.teamcode.doge_cv;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;

public class SuperWebcamName{
    public OpenCvWebcam openCvWebcam;
    public WebcamName webcamName;

    public SuperWebcamName(OpenCvWebcam openCvWebcam, WebcamName webcamName){
        this.openCvWebcam = openCvWebcam;
        this.webcamName = webcamName;
    }
}
