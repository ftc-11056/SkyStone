package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot extends OpMode {

    /* Drive Train Motor */
    private DcMotor LB = null;
    private DcMotor  LF = null;
    private DcMotor  RF = null;
    private DcMotor  RB = null;

    /*IMU Fileds*/
    protected BNO055IMU IMU = null;
    protected Orientation angles = null;


    /*Mechanisms*/
    protected DriveTrain MyDriveTrain = null;



    @Override
    public void init() {

        // Define and Initialize Motors Of Drive Train
        LB  = hardwareMap.get(DcMotor.class, "LB");
        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF  = hardwareMap.get(DcMotor.class, "RF");
        RB  = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        LB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*Define and Initialize Of IMU*/

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        IMU.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.

        angles = IMU.getAngularOrientation();

        // Define and initialize ALL installed servos.

        // Define Mechanisms:
        MyDriveTrain = new DriveTrain(LB,LF, RF, RB);
    }

    @Override
    public void loop() {

    }
}
