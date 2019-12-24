package org.firstinspires.ftc.teamcode.doge_cv.afterDoge;

import org.opencv.core.Mat;

public abstract class OpenCvPipeline
{
    public abstract Mat processFrame(Mat input);
    public void onViewportTapped() {}
}