/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Note: credit where credit is due - some parts of OpenCv's
 *       JavaCameraView were used as a reference
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.easyopencv;

import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraBase;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.io.IOException;
import java.util.List;

class OpenCvInternalCameraImpl extends OpenCvCameraBase implements Camera.PreviewCallback, org.firstinspires.ftc.teamcode.easyopencv.OpenCvInternalCamera
{
    private Camera camera;
    private CameraDirection direction;
    private Mat rawSensorMat;
    private Mat rgbMat;
    private SurfaceTexture bogusSurfaceTexture;
    private int maxZoom = -1;
    private volatile boolean isOpen = false;
    private volatile boolean isStreaming = false;

    public OpenCvInternalCameraImpl(CameraDirection direction)
    {
        this.direction = direction;
    }

    public OpenCvInternalCameraImpl(CameraDirection direction, int containerLayoutId)
    {
        super(containerLayoutId);
        this.direction = direction;
    }

    @Override
    public org.openftc.easyopencv.OpenCvCameraRotation getDefaultRotation()
    {
        return org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;
    }

    @Override
    protected int mapRotationEnumToOpenCvRotateCode(org.openftc.easyopencv.OpenCvCameraRotation rotation)
    {
        /*
         * The camera sensor in a phone is mounted sideways, such that the raw image
         * is only upright when the phone is rotated to the left. Therefore, we need
         * to manually rotate the image if the phone is in any other orientation
         */

        if(direction == CameraDirection.BACK)
        {
            if(rotation == org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT)
            {
                return Core.ROTATE_90_CLOCKWISE;
            }
            else if(rotation == org.openftc.easyopencv.OpenCvCameraRotation.UPSIDE_DOWN)
            {
                return Core.ROTATE_90_COUNTERCLOCKWISE;
            }
            else if(rotation == org.openftc.easyopencv.OpenCvCameraRotation.SIDEWAYS_RIGHT)
            {
                return Core.ROTATE_180;
            }
            else
            {
                return -1;
            }
        }
        else if(direction == CameraDirection.FRONT)
        {
            if(rotation == org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT)
            {
                return Core.ROTATE_90_COUNTERCLOCKWISE;
            }
            else if(rotation == org.openftc.easyopencv.OpenCvCameraRotation.UPSIDE_DOWN)
            {
                return Core.ROTATE_90_CLOCKWISE;
            }
            else if(rotation == org.openftc.easyopencv.OpenCvCameraRotation.SIDEWAYS_RIGHT)
            {
                return Core.ROTATE_180;
            }
            else
            {
                return -1;
            }
        }

        return -1;
    }

    @Override
    protected void stopStreamingImplSpecific() {

    }


    @Override
    protected void openCameraDeviceImplSpecific() {

    }

    @Override
    protected void closeCameraDeviceImplSpecific() {

    }

    @Override
    protected void startStreamingImplSpecific(int width, int height) {

    }

    /*
     * This needs to be synchronized with stopStreamingImplSpecific()
     * because we touch objects that are destroyed in that method.
     */
    @Override
    public synchronized void onPreviewFrame(byte[] data, Camera camera)
    {
        notifyStartOfFrameProcessing();

        /*
         * Unfortunately, we can't easily create a Java byte[] that
         * references the native memory in a Mat, so we have to do
         * a memcpy from our Java byte[] to the native one in the Mat.
         * (If we could, then we could have the camera dump the preview
         * image directly into the Mat).
         *
         * TODO: investigate using a bit of native code to remove the need to do a memcpy
         */
        if(rawSensorMat != null)
        {
            rawSensorMat.put(0,0,data);

            Imgproc.cvtColor(rawSensorMat, rgbMat, Imgproc.COLOR_YUV2RGBA_NV21, 4);
            handleFrame(rgbMat);

            if(camera != null)
            {
                camera.addCallbackBuffer(data);
            }
        }
    }

    @Override
    public void startStreaming(int width, int height, OpenCvCameraRotation rotation, BufferMethod bufferMethod) {

    }

    @Override
    public synchronized void setFlashlightEnabled(boolean enabled)
    {
        if(!isOpen || camera == null)
        {
            throw new org.openftc.easyopencv.OpenCvCameraException("Cannot control flash until camera is opened!");
        }
        else
        {
            Camera.Parameters parameters = camera.getParameters();

            List<String> supportedFlashModes = parameters.getSupportedFlashModes();

            if(supportedFlashModes == null)
            {
                throw new org.openftc.easyopencv.OpenCvCameraException("Camera does not have a flash!");
            }
            else if(!supportedFlashModes.contains(Camera.Parameters.FLASH_MODE_TORCH))
            {
                throw new org.openftc.easyopencv.OpenCvCameraException("Camera flash does not support torch mode!");
            }

            if(enabled)
            {
                parameters.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
            }
            else
            {
                parameters.setFlashMode(Camera.Parameters.FLASH_MODE_OFF);
            }

            camera.setParameters(parameters);
        }
    }

    @Override
    public synchronized int getMaxSupportedZoom()
    {
        if(!isOpen || camera == null)
        {
            throw new org.openftc.easyopencv.OpenCvCameraException("Cannot get supported zooms until camera is opened and streaming is started");
        }
        else
        {
            if(maxZoom == -1)
            {
                throw new org.openftc.easyopencv.OpenCvCameraException("Cannot get supported zooms until streaming has been started");
            }

            return maxZoom;
        }
    }

    @Override
    public synchronized void setZoom(int zoom)
    {
        if(!isOpen || camera == null)
        {
            throw new org.openftc.easyopencv.OpenCvCameraException("Cannot set zoom until camera is opened and streaming is started");
        }
        else
        {
            if(maxZoom == -1)
            {
                throw new org.openftc.easyopencv.OpenCvCameraException("Cannot set zoom until streaming has been started");
            }
            else if(zoom > maxZoom)
            {
                throw new org.openftc.easyopencv.OpenCvCameraException(String.format("Zoom value of %d requested, but maximum zoom supported in current configuration is %d", zoom, maxZoom));
            }
            else if(zoom < 0)
            {
                throw new org.openftc.easyopencv.OpenCvCameraException("Zoom value cannot be less than 0");
            }
            Camera.Parameters parameters = camera.getParameters();
            parameters.setZoom(zoom);
            camera.setParameters(parameters);
        }
    }

    @Override
    public synchronized void setRecordingHint(boolean hint)
    {
        if(!isOpen || camera == null)
        {
            throw new org.openftc.easyopencv.OpenCvCameraException("Cannot set recording hint until camera is opened");
        }
        else
        {
            Camera.Parameters parameters = camera.getParameters();
            parameters.setRecordingHint(hint);
            camera.setParameters(parameters);
        }
    }

    @Override
    public synchronized void setHardwareFrameTimingRange(FrameTimingRange frameTiming)
    {
        if(!isOpen || camera == null)
        {
            throw new org.openftc.easyopencv.OpenCvCameraException("Cannot set hardware frame timing range until camera is opened");
        }
        else
        {
            Camera.Parameters parameters = camera.getParameters();
            parameters.setPreviewFpsRange(frameTiming.min*1000, frameTiming.max*1000);
            camera.setParameters(parameters);
        }
    }

    @Override
    public synchronized FrameTimingRange[] getFrameTimingRangesSupportedByHardware()
    {
        if(!isOpen || camera == null)
        {
            throw new OpenCvCameraException("Cannot get frame timing ranges until camera is opened");
        }
        else
        {
            Camera.Parameters parameters = camera.getParameters();
            List<int[]> rawRanges = parameters.getSupportedPreviewFpsRange();
            FrameTimingRange[] ranges = new FrameTimingRange[rawRanges.size()];

            for(int i = 0; i < ranges.length; i++)
            {
                int[] raw = rawRanges.get(i);
                ranges[i] = new FrameTimingRange(raw[0]/1000, raw[1]/1000);
            }

            return ranges;
        }
    }
}
