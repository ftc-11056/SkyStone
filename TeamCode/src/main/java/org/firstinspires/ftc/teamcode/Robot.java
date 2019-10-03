package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();

    /* Drive Train Motor */
    public DcMotor LB = null;
    private DcMotor  LF = null;
    public DcMotor  RF = null;
    public DcMotor  RB = null;

    /*Systems Motor and Servo*/
    public DcMotor LinearMotor = null;
    public DcMotor IntakeL = null;
    public DcMotor IntakeR = null;
    public Servo Arm = null;
    public Servo Output = null;

    /*IMU Fileds*/
    protected BNO055IMU IMU = null;
    protected Orientation angles = null;


    /*Mechanisms*/
    protected DriveTrain MyDriveTrain = null;
    protected Odometry MyOdometry = null;

    public double servoPosition = 0.005;

    @Override
    public void runOpMode() throws InterruptedException {

        runtime.reset();

        // Define and Initialize Motors Of Drive Train
        LB  = hardwareMap.get(DcMotor.class, "LB");
        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF  = hardwareMap.get(DcMotor.class, "RF");
        RB  = hardwareMap.get(DcMotor.class, "RB");
        // Define and Initialize Systems Motors and Servo
        LinearMotor = hardwareMap.get(DcMotor.class,"LinearMotor");
        Arm  = hardwareMap.get(Servo.class, "Arm");
        Output  = hardwareMap.get(Servo.class, "OutPut");
        IntakeL = hardwareMap.get(DcMotor.class,"IntakeL");
        IntakeR = hardwareMap.get(DcMotor.class,"IntakeR");



        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        IntakeL.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE the intake System


        // Set all motors to zero power
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
      //  linear_motor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
       /* LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

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
        MyDriveTrain = new DriveTrain(LB,LF, RF, RB);
    }


}
