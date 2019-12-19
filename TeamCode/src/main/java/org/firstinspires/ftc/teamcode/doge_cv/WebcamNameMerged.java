package org.firstinspires.ftc.teamcode.doge_cv;

import android.content.Context;
import android.hardware.usb.UsbManager;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.internal.camera.RenumberedCameraFrame;
import org.firstinspires.ftc.robotcore.internal.camera.libuvc.api.UvcApiCameraFrame;
import org.firstinspires.ftc.robotcore.internal.camera.libuvc.nativeobject.UvcFrame;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraBase;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.lang.reflect.Field;

public class WebcamNameMerged extends OpenCvCameraBase implements CameraCaptureSession.CaptureCallback, HardwareDevice, CameraName {

//    private final CameraManagerInternal cameraManager;
//    private final Executor serialThreadPool;
    private final int secondsPermissionTimeout = 1;
    private final CameraName cameraName;
    private CameraCharacteristics cameraCharacteristics = null;
    private Camera camera = null;
    private CameraCaptureSession cameraCaptureSession = null;
    private Mat rawSensorMat;
    private Mat rgbMat;
    private byte[] imgDat;


    public WebcamNameMerged(/*CameraManagerInternal cameraManager, Executor serialThreadPool,*/ CameraName cameraName) {
        /*this.cameraManager = cameraManager;
        this.serialThreadPool = serialThreadPool;*/
        this.cameraName = cameraName;
    }

    public synchronized ExposureControl getExposureControl()
    {
        ExposureControl control = camera.getControl(ExposureControl.class);

        if(control == null)
        {
            throw new RuntimeException("Exposure control not supported!");
        }

        return control;
    }

    public synchronized FocusControl getFocusControl()
    {
        FocusControl control = camera.getControl(FocusControl.class);

        if(control == null)
        {
            throw new RuntimeException("Focus control not supported!");
        }

        return control;
    }

    @Override
    public boolean isWebcam() {
        return false;
    }

    @Override
    public boolean isCameraDirection() {
        return false;
    }

    @Override
    public boolean isSwitchable() {
        return false;
    }

    @Override
    public boolean isUnknown() {
        return false;
    }

    @Override
    public void asyncRequestCameraPermission(Context context, Deadline deadline, Continuation<? extends Consumer<Boolean>> continuation) {

    }

    @Override
    public boolean requestCameraPermission(Deadline deadline) {
        return false;
    }

    public synchronized CameraCharacteristics getCameraCharacteristics()
    {
        return cameraCharacteristics;
    }

    /*@Override
    protected void openCameraDeviceImplSpecific() {
        try
        {
            camera = cameraManager.requestPermissionAndOpenCamera(new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS), cameraName, null);

            if (camera != null) //Opening succeeded!
            {
                cameraCharacteristics = camera.getCameraName().getCameraCharacteristics();
            }
            else //Opening failed! :(
            {
                cameraCharacteristics = cameraName.getCameraCharacteristics();
            }
        }
        catch (Exception e)
        {
            camera = null;
            throw e;
        }
    }*/

    @Override
    protected void openCameraDeviceImplSpecific() {

    }

    @Override
    public synchronized void closeCameraDeviceImplSpecific()
    {
        if (camera != null)
        {
            stopStreaming();
            camera.close();
            camera = null;
        }
    }

    @Override
    protected void startStreamingImplSpecific(int width, int height) {

    }

    /*public synchronized void startStreamingImplSpecific(final int width, final int height)
    {
        final CountDownLatch captureStartResult = new CountDownLatch(1);

        boolean sizeSupported = false;
        for(Size s : cameraCharacteristics.getSizes(ImageFormat.YUY2))
        {
            if(s.getHeight() == height && s.getWidth() == width)
            {
                sizeSupported = true;
                break;
            }
        }

        if(!sizeSupported)
        {
            throw new OpenCvCameraException("Camera does not support requested resolution!");
        }

        try
        {
            camera.createCaptureSession(Continuation.create(serialThreadPool, new CameraCaptureSession.StateCallback()
            {
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session)
                {
                    try
                    {
                        CameraMode streamingMode = new CameraMode(width, height, 30, FrameFormat.YUYV);

                        //Indicate how we want to stream
                        final CameraCaptureRequest cameraCaptureRequest = camera.createCaptureRequest(
                                streamingMode.getAndroidFormat(),
                                streamingMode.getSize(),
                                streamingMode.getFramesPerSecond());

                        // Start streaming!
                        session.startCapture(cameraCaptureRequest,
                                WebcamName.this,
                                Continuation.create(serialThreadPool, new CameraCaptureSession.StatusCallback()
                                {
                                    @Override
                                    public void onCaptureSequenceCompleted(
                                            @NonNull CameraCaptureSession session,
                                            CameraCaptureSequenceId cameraCaptureSequenceId,
                                            long lastFrameNumber)
                                    {
                                        RobotLog.d("capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                    }
                                }));
                    }
                    catch (CameraException | RuntimeException e)
                    {
                        e.printStackTrace();
                        RobotLog.e("exception setting repeat capture request: closing session: %s", session);
                        session.close();
                        session = null;
                    }

                    System.out.println("OpenCvWebcam: onConfigured");
                    cameraCaptureSession = session;
                    captureStartResult.countDown();
                }

                @Override
                public void onClosed(@NonNull CameraCaptureSession session)
                {

                }
            }));
        }
        catch (CameraException | RuntimeException e)
        {
            System.out.println("OpenCvWebcam: exception starting capture");
            captureStartResult.countDown();
        }

        // Wait for the above to complete
        try
        {
            captureStartResult.await(1, TimeUnit.SECONDS);
            System.out.println("OpenCvWebcam: streaming started");
        }
        catch (InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
    }*/

    @Override
    protected OpenCvCameraRotation getDefaultRotation()  {
        return OpenCvCameraRotation.SIDEWAYS_LEFT;
    }

    @Override
    protected int mapRotationEnumToOpenCvRotateCode(OpenCvCameraRotation rotation) {
        /*
         * The camera sensor in a webcam is mounted in the logical manner, such
         * that the raw image is upright when the webcam is used in its "normal"
         * orientation. However, if the user is using it in any other orientation,
         * we need to manually rotate the image.
         */

        if(rotation == OpenCvCameraRotation.SIDEWAYS_LEFT)
        {
            return Core.ROTATE_90_COUNTERCLOCKWISE;
        }
        if(rotation == OpenCvCameraRotation.SIDEWAYS_RIGHT)
        {
            return Core.ROTATE_90_CLOCKWISE;
        }
        else if(rotation == OpenCvCameraRotation.UPSIDE_DOWN)
        {
            return Core.ROTATE_180;
        }
        else
        {
            return -1;
        }
    }

    public synchronized void stopStreamingImplSpecific()
    {
        imgDat = null;
        rgbMat = null;
        rawSensorMat = null;

        if (cameraCaptureSession != null)
        {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    @Override
    public synchronized void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame)
    {
        notifyStartOfFrameProcessing();

        if(imgDat == null)
        {
            imgDat = new byte[cameraFrame.getImageSize()];
        }
        if(rgbMat == null)
        {
            rgbMat = new Mat(cameraFrame.getSize().getHeight(), cameraFrame.getSize().getWidth(), CvType.CV_8UC1);
        }
        if(rawSensorMat == null)
        {
            rawSensorMat = new Mat(cameraFrame.getSize().getHeight(), cameraFrame.getSize().getWidth(), CvType.CV_8UC2);
        }

        try
        {
            /*
             * v1.2 HOTFIX for renderscript crashes on some devices when using frame.copyToBitmap()
             *
             * Is it pretty? Heck no. Does it work? Yes! :)
             * Also it seems to be *considerably* more efficient than copyToBitmap(). Not entirely
             * sure why because one would think renderscript would be faster since it can run on
             * the GPU...
             */
            RenumberedCameraFrame renumberedCameraFrame = (RenumberedCameraFrame) cameraFrame;
            Field innerFrameField = RenumberedCameraFrame.class.getDeclaredField("innerFrame");
            innerFrameField.setAccessible(true);
            CameraFrame innerFrame = (CameraFrame) innerFrameField.get(renumberedCameraFrame);
            UvcApiCameraFrame uvcApiCameraFrame = (UvcApiCameraFrame) innerFrame;
            Field uvcFrameField = UvcApiCameraFrame.class.getDeclaredField("uvcFrame");
            uvcFrameField.setAccessible(true);
            UvcFrame uvcFrame = (UvcFrame) uvcFrameField.get(uvcApiCameraFrame);

            uvcFrame.getImageData(imgDat);
            rawSensorMat.put(0,0,imgDat);
            Imgproc.cvtColor(rawSensorMat, rgbMat, Imgproc.COLOR_YUV2RGBA_YUY2, 4);

            handleFrame(rgbMat);

        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    /**
     * Returns the USB serial number of the webcam
     *
     * @return the USB serial number of the webcam
     */
    @NonNull
    SerialNumber getSerialNumber() {
        return null;
    }

    /**
     * Returns the USB device path currently associated with this webcam.
     * May be null if the webcam is not presently attached.
     *
     * @return returns the USB device path associated with this name.
     * @see UsbManager#getDeviceList()
     */
    @Nullable
    String getUsbDeviceNameIfAttached() {
        return null;
    }

    /**
     * Returns whether this camera currently attached to the robot controller
     *
     * @return whether this camera currently attached to the robot controller
     */
    boolean isAttached() {
        return false;
    }

}
