package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="estFieldOrientedMecanumDrive", group="teamcode")
public class estFieldOrientedMecanumDrive extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles = new Orientation();
    double left;
    double right;
    double drive;
    int mode = 1;
    double turn;
    double max;
    DcMotor LFmotor;
    DcMotor RFmotor;
    DcMotor RBmotor;
    DcMotor LBmotor;
    /* Declare OpMode members. */
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
    HardwareMap hwMap = null;

    @Override
    public void runOpMode () throws InterruptedException {

        LFmotor = hardwareMap.get(DcMotor.class, "LFmotor");
        RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
        LBmotor = hardwareMap.get(DcMotor.class, "LBmotor");
        RBmotor = hardwareMap.get(DcMotor.class, "RBmotor");

        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LBmotor.setDirection(DcMotor.Direction.REVERSE);
        LFmotor.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();




        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        // robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

//        imu.getAcceleration();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            angles = imu.getAngularOrientation();
            double heading = angles.firstAngle;

            if (gamepad1.right_bumper) mode = 2;
            else if (gamepad1.a) mode = 1;

            if (mode == 2) {
                arcade(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, LFmotor, RFmotor, LBmotor, RBmotor);
            }
            else {
                fieldOriented(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, heading, LFmotor, RFmotor, LBmotor, RBmotor);
            }
        }
    }
    public void arcade(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = -y + x + c;
        double rightFrontVal = -y - x - c;
        double leftBackVal = -y - x + c;
        double rightBackVal = -y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }

        leftFront.setPower(leftFrontVal);
        rightFront.setPower(rightFrontVal);
        leftBack.setPower(leftBackVal);
        rightBack.setPower(rightBackVal);
    }
    public void fieldOriented(double y, double x, double c, double gyroheading, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double cosA = Math.cos(gyroheading);
        double sinA = Math.sin(gyroheading);
        double xOut = x * cosA - y * sinA;
        double yOut = x * sinA + y * cosA;
        arcade(yOut, xOut, c, leftFront, rightFront, leftBack, rightBack);
    }

}


