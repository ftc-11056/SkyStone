package org.firstinspires.ftc.teamcode.doge_cv.afterDoge;

import org.opencv.core.Mat;

public abstract class OpenCvTracker
{
    private Mat mat = new Mat();

    public abstract Mat processFrame(Mat input);

    protected final Mat processFrameInternal(Mat input)
    {
        input.copyTo(mat);
        return processFrame(mat);
    }
}