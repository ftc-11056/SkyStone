package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class Robot extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();

    /* Drive Train Motor */
    public DcMotor LB = null;
    public DcMotor  LF = null;
    public DcMotor  RF = null;
    public DcMotor  RB = null;

    /*Systems Motor and Servo*/
    public DcMotor leftLinearMotor = null;
    public DcMotor rightLinearMotor = null;
    public DcMotor IntakeL = null;
    public DcMotor IntakeR = null;
    public Servo Arm = null;
    public Servo Output = null;
    public Servo LeftServo = null;
    public Servo RightServo = null;

    /*IMU Fileds*/
    protected BNO055IMU IMU = null;
    protected Orientation angles = null;

    //digitalChannels
    public DigitalChannel downMagnetElevator = null;
    public DigitalChannel upMagnetElevator = null;


    /*Mechanisms*/
    protected DriveTrain MyDriveTrain = null;
    protected Odometry MyOdometry = null;
    protected IntakeTrain MyIntake = null;
    protected VuforiaStone MyVuforiaStone = null;
    protected elevator MyElevator = null;

    public double servoPosition = 0.005;
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

        runtime.reset();

        // Define and Initialize Motors Of Drive Train
        LB  = hardwareMap.get(DcMotor.class, "LB");
        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF  = hardwareMap.get(DcMotor.class, "RF");
        RB  = hardwareMap.get(DcMotor.class, "RB");
        // Define and Initialize Systems Motors and Servo
        leftLinearMotor = hardwareMap.get(DcMotor.class,"leftLinearMotor");
        rightLinearMotor = hardwareMap.get(DcMotor.class,"rightLinearMotor");
        Arm  = hardwareMap.get(Servo.class, "Arm");
        Output  = hardwareMap.get(Servo.class, "OutPut");
        IntakeL = hardwareMap.get(DcMotor.class,"IntakeL");
        IntakeR = hardwareMap.get(DcMotor.class,"IntakeR");

      //  LeftServo  = hardwareMap.get(Servo.class, "LeftServo");
      //  RightServo  = hardwareMap.get(Servo.class, "RightServo");

        upMagnetElevator = hardwareMap.get(DigitalChannel.class,"upMagnetELevator");
        downMagnetElevator = hardwareMap.get(DigitalChannel.class,"downMagnetELevator");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        IntakeL.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE the intake System
//        leftLinearMotor.setDirection(DcMotor.Direction.REVERSE);//set to rverse the elevator system

        leftLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Set all motors to zero power
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
      //  linear_motor.setPower(0);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVu = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parametersVu.vuforiaLicenseKey = VUFORIA_KEY;
        parametersVu.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parametersVu);
        //VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
       /* LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);/*

        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*Define and Initialize Of IMU*/
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        IMU.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStarted() && !isStopRequested() && !IMU.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        // Define and initialize ALL installed servos.

        // Define Mechanisms:
        MyDriveTrain = new DriveTrain(LB,LF, RF, RB, IMU);
        MyIntake = new IntakeTrain(IntakeL, IntakeR);
        MyVuforiaStone=new VuforiaStone( webcamName,parametersVu, targetsSkyStone, vuforia,lastLocation);
        MyElevator = new elevator(leftLinearMotor, rightLinearMotor, upMagnetElevator, downMagnetElevator);
    }

    //methodes:

    public void autoOpenWitheStone(boolean reason){
        boolean [] upStep;
        upStep = new boolean[5];
        upStep[1] = true;

        if (reason) {
            if (upStep[1] == true){
                Output.setPosition(0.75);
                upStep[2] = true;
            }
            if (upStep[2] == true && Output.getPosition() > 0.6){
                MyElevator.setPower(1,1);
                upStep[3] = true;
                upStep[2] = false;
            }
            if (upStep[3] = true && upMagnetElevator.getState() == false){
                MyElevator.setPower(0,0);
                Arm.setPosition(0.104020);
                upStep[4] = true;
                upStep[3] = false;
            }
            if (upStep[4] == true && Arm.getPosition() > 0.09){
                MyElevator.setPower(1,1);
            }
        }
    }
}
