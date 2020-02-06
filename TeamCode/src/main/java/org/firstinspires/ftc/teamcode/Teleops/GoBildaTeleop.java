package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.PP.Odometry;
import org.firstinspires.ftc.teamcode.Systems.CustomadeIntake;
import org.firstinspires.ftc.teamcode.Systems.DriveTrain;
import org.firstinspires.ftc.teamcode.Systems.IntakeTrain;
import org.firstinspires.ftc.teamcode.Systems.elevator;

@TeleOp(name = "GoBildaTeleop", group = "teleops")
public class GoBildaTeleop extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();

    //Drive Train Motor
    protected CustomadeIntake MyIntake = null;
    public DcMotor LB = null;
    public DcMotor LF = null;
    public DcMotor RF = null;
    public DcMotor RB = null;
    public DcMotor IntakeL = null;
    public DcMotor IntakeR = null;

    /*IMU Fileds*/
    protected BNO055IMU IMU = null;

    protected Orientation angles = null;

    /*Mechanisms*/
    protected DriveTrain MyDriveTrain = null;

    private double leftStickX = 0;
    private double leftStickY = 0;
    private double rightStickX = 0;




    @Override

    public void runOpMode() throws InterruptedException {
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        IntakeL = hardwareMap.get(DcMotor.class, "IntakeL");
        IntakeR = hardwareMap.get(DcMotor.class, "IntakeR");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        MyDriveTrain = new DriveTrain(LB, LF, RF, RB, IMU);
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        // make sure the imu gyro is calibrated before continuing.
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStarted() && !isStopRequested() && !IMU.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

         while (!isStarted()) {
            runtime.reset();
        }
        waitForStart();
        while (opModeIsActive()) {

            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = angles.firstAngle;

            if (gamepad1.x) MyDriveTrain.setMode("arcade");
            else if (gamepad1.b) MyDriveTrain.setMode("Oriented");

            if (MyDriveTrain.getMode().equals("Oriented")) {
                MyDriveTrain.fieldOriented(-leftStickY, leftStickX, rightStickX, -heading);
            }
            else { MyDriveTrain.arcade(-leftStickY, leftStickX, rightStickX);
            }

            if (gamepad1.right_trigger > 0) {
                leftStickX = gamepad1.left_stick_x * 0.5;
                leftStickY = gamepad1.left_stick_y * 0.5;
                rightStickX = gamepad1.right_stick_x * 0.5;
            } else {
                leftStickX = gamepad1.left_stick_x;
                leftStickY = gamepad1.left_stick_y;
                rightStickX = gamepad1.right_stick_x;

                if (gamepad1.dpad_up) {
                    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                    parameters.loggingEnabled = true;
                    parameters.loggingTag = "IMU";
                    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                    IMU.initialize(parameters);
                }
                if (!isStopRequested() && !IMU.isGyroCalibrated()) {
                    idle();
                    telemetry.addLine("imu isnt calibrated");

                }
                if (gamepad1.right_trigger > 0) {
                    IntakeL.setPower(1);
                    IntakeR.setPower(-1);
                } else if (gamepad1.left_trigger > 0) {
                    IntakeL.setPower(-1);
                    IntakeR.setPower(1);
                } else {
                    IntakeL.setPower(0);
                    IntakeR.setPower(0);  }
        }
    }
}
}