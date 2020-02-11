package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.PP.OdometryCustomMade;
import org.firstinspires.ftc.teamcode.PP.OurPoint;
import org.firstinspires.ftc.teamcode.PP.PurePursuitGUI;
import org.firstinspires.ftc.teamcode.Systems.CustomadeIntake;
import org.firstinspires.ftc.teamcode.Systems.DriveTrain;
import org.firstinspires.ftc.teamcode.Systems.IntakeTrain;
import org.firstinspires.ftc.teamcode.Systems.elevator;


public class RobotCustomade extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();

    public FtcDashboard dashboard;
    public TelemetryPacket packet = null;

    //Drive Train Motor
    public DcMotor LB = null;
    public DcMotor LF = null;
    public DcMotor RF = null;
    public DcMotor RB = null;

    //Systems Motor and Servo
    public DcMotor leftLinearMotor = null;
    public DcMotor rightLinearMotor = null;
    public DcMotor IntakeL = null;
    public DcMotor IntakeR = null;
    public Servo Arm = null;
    public Servo Output = null;
    public Servo LeftServo = null;
    public Servo RightServo = null;
    public Servo ParkingMot = null;
    public Servo Capstone = null;
    public DigitalChannel LeftTouch = null;
    public DigitalChannel RightTouch = null;


    /*IMU Fileds*/
    protected BNO055IMU IMU = null;
    protected Orientation angles = null;
    protected Velocity Velocity;
    protected Acceleration Acceleration;

    //digitalChannels
    public DigitalChannel downMagnetElevator = null;
    public DigitalChannel upMagnetElevator = null;

    /*Analog Sensor*/
    protected DistanceSensor cubeIn;


    /*Mechanisms*/
    protected DriveTrain MyDriveTrain = null;
    protected OdometryCustomMade MyOdometry = null;
    protected CustomadeIntake MyIntake = null;
    protected elevator MyElevator = null;

    public double servoPosition = 0.005;
    protected int fixedPosition = 0;

    public double OutputClose = 0.75;
    public double OutputOpen = 0.53 ;

    public double CapstoneUp = 0.44;
    public double CapstoneDown = 0.9;

    public double ParkingMotOut = 1;
    public double ParkingMotIn = 0.33;

    public double LeftServoDown = 0.79;
    public double RightServoDown = 0.24;
    public double LeftServoUp = 0.24;
    public double RightServoUp = 0.73;
    public double LeftServoMiddle = 0.515;
    public double RightServoMiddle = 0.487;

    public double ArmClose = 0.79;
    public double ArmOpen = 0.3;
    public boolean IntakeStop = true;
    public int PointIndexStartElavator = 0;
    public double PlacingStoneTime = 0;
    public double Delta;
    public double cubeNotInMM = 100;

    public String passWord = "dont pass";

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    @Override
    public void runOpMode() throws InterruptedException {

        runtime.reset();

        // Define and Initialize Motors Of Drive Train
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");

        //Define and Initialize Systems Motors and Servo
        leftLinearMotor = hardwareMap.get(DcMotor.class, "leftLinearMotor");
        rightLinearMotor = hardwareMap.get(DcMotor.class, "rightLinearMotor");
        Arm = hardwareMap.get(Servo.class, "Arm");

        Output = hardwareMap.get(Servo.class, "OutPut");

        IntakeL = hardwareMap.get(DcMotor.class, "IntakeL");
        IntakeR = hardwareMap.get(DcMotor.class, "IntakeR");

        ParkingMot = hardwareMap.get(Servo.class, "ParkingMot");

        Capstone = hardwareMap.get(Servo.class, "Capstone");

        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");

        upMagnetElevator = hardwareMap.get(DigitalChannel.class, "upMagnetELevator");
        downMagnetElevator = hardwareMap.get(DigitalChannel.class, "downMagnetELevator");

        LeftTouch = hardwareMap.get(DigitalChannel.class, "LeftTouch");
        RightTouch = hardwareMap.get(DigitalChannel.class, "RightTouch");

        cubeIn = hardwareMap.get(DistanceSensor.class, "cubeIn");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);

        leftLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Odometry Encoders:
        IntakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        IntakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set all motors to zero power
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Define and Initialize Of IMU

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
//        IMU.initialize(parameters);
/*
        // make sure the imu gyro is calibrated before continuing.
        while (!isStarted() && !isStopRequested() && !IMU.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        */
        /*angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        Velocity = IMU.getVelocity();
        Acceleration = IMU.getAcceleration();
*/

        // Define and initialize ALL installed servos.
        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);
        Output.setPosition(OutputOpen);
        Arm.setPosition(ArmClose);


        //Define Mechanisms:
        MyDriveTrain = new DriveTrain(LB, LF, RF, RB, IMU/*,Touch_foundation*/);
        MyIntake = new CustomadeIntake(IntakeL, IntakeR);
        MyElevator = new elevator(leftLinearMotor, rightLinearMotor, upMagnetElevator, downMagnetElevator, fixedPosition);
    }

    public void setPointIndexStartElavator(int pointIndexStartElavator) {
        PointIndexStartElavator = pointIndexStartElavator;
    }

    public void updateOdometry() {
        double currentTime = runtime.seconds();
        double odometryRight = IntakeR.getCurrentPosition();
        double odometryLeft = (IntakeL.getCurrentPosition());
        double odometryHorizental = -(rightLinearMotor.getCurrentPosition());
        MyOdometry.setAll(odometryRight, odometryLeft, odometryHorizental, currentTime);
    }

    public void BuildOdometry(OurPoint Position) {
        MyOdometry = new OdometryCustomMade(Position);
    }

    public void StopAndSavePlace(PurePursuitGUI MyPurePursuitGUI, FtcDashboard dashboard) {
        double stopTime = runtime.seconds();
        TelemetryPacket packet;
        MyDriveTrain.setStopPosition(new OurPoint(MyOdometry.dSide, MyOdometry.dForward));
        while (((runtime.seconds() - stopTime) < 1.5) && opModeIsActive()) {
            packet = new TelemetryPacket();
            if (MyPurePursuitGUI.lost) {
                packet.addLine("lost");
            }
            packet.addLine("stop");
            MyDriveTrain.encoderStop(new OurPoint(MyOdometry.dSide, MyOdometry.dForward));
            updateOdometry();
            MyPurePursuitGUI.updateGraghic(packet);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public void PlacingStoneWhitPoints(int currentPointIndex, TelemetryPacket packet) {
        int delta = currentPointIndex - PointIndexStartElavator;
        packet.put("delte", delta);
        if (delta >= 0 && delta <= 4) {
            MyElevator.ElevateWithEncoder(-350, 0.4, 0.3);
        }
        if (delta >= 5 && delta <= 12) {
            Arm.setPosition(ArmOpen);
        }
        if (delta >= 13 && delta <= 18) {
            MyElevator.ElevateWithEncoder(0, 0.7, 0.0035);
        }
        if (delta >= 19 && delta <= 23) {
            Output.setPosition(OutputOpen);
        }
        if (delta >= 24 && delta <= 32) {
            MyElevator.ElevateWithEncoder(-350, 0.4, 0.3);
        }
        if (delta >= 33 && delta <= 38) {
            Arm.setPosition(ArmClose);
        }
        if (delta >= 39 && delta <= 42) {
            MyElevator.ElevateWithEncoder(0, 0.7, 0.0035);
        }
    }

    public boolean PlacingStoneWhitTime() {
        boolean ElvateBusy = true;
        Delta = runtime.seconds() - PlacingStoneTime;
        if (Delta >= 0 && Delta <= 0.7) {
            Arm.setPosition(ArmOpen);
        }
        if (Delta > 0.7 && Delta <= 1.6) {
            Output.setPosition(OutputOpen);
        }
        if (Delta > 1.6 && Delta <= 2.5) {
            Arm.setPosition(ArmClose);
            ElvateBusy = false;
        }
        return ElvateBusy;
    }

    public void TouchFoundation (DigitalChannel left, DigitalChannel right){
        double Ftime = runtime.seconds();
        while (opModeIsActive() && left.getState() && right.getState() && (runtime.seconds() -Ftime  <= 0.5)){
            updateOdometry();
            double power = -0.3;
            double cPower = IMUError(180, 1);
            MyDriveTrain.arcade(power,0,cPower);
        }
        MyDriveTrain.SetPower(0,0,0,0);
    }

    public double IMUError(double TargetAngle, double turnSpeed){
        return turnSpeed*Range.clip((Math.toRadians(MyDriveTrain.getAngle()) - (Math.toRadians(TargetAngle)))/ Math.toRadians(30), -1, 1);
    }

    public void RotateP1(int degrees, double power, double timeoutR,double KP) {
        if (MyDriveTrain.getAngle() < degrees) {
            while (MyDriveTrain.getAngle() < degrees && opModeIsActive()) {
//                SumErrors = SumErrors + (getAngle() + degrees);
                double error = degrees-MyDriveTrain.getAngle();
                MyDriveTrain.LeftFront.setPower(-power  * error * KP);
                MyDriveTrain.LeftBack.setPower(-power  * error * KP);
                MyDriveTrain.RightFront.setPower(power  * error * KP);
                MyDriveTrain.RightBack.setPower(power  * error * KP);
                updateOdometry();
            }
        } else if (MyDriveTrain.getAngle() > degrees) {
            while (MyDriveTrain.getAngle() > degrees && opModeIsActive()) {
                double error = MyDriveTrain.getAngle()-degrees;
                MyDriveTrain.LeftFront.setPower(power  * error * KP);
                MyDriveTrain.LeftBack.setPower(power * error * KP);
                MyDriveTrain.RightFront.setPower(-power * error * KP);
                MyDriveTrain.RightBack.setPower(-power  * error * KP);
                updateOdometry();
            }
        } else {
            return;
        }
        // turn the motors off.
        MyDriveTrain.LeftFront.setPower(0);
        MyDriveTrain.LeftBack.setPower(0);
        MyDriveTrain.RightFront.setPower(0);
        MyDriveTrain.RightBack.setPower(0);

    }

}

