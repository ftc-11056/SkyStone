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
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public DigitalChannel Touch_Foundation = null;


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

    public double OutputClose = 0.7;
    public double OutputOpen = 0.4;

    public double CapstoneUp = 0.15  ;
    public double CapstoneDown = 0.54;


    public double ParkingMotIn = 0.6;
    public double ParkingMotOut = 0;

    public double RightServoDown = 0;
    public double LeftServoDown = 1;
    public double RightServoUp = 0.75;
    public double LeftServoUp = 0.25;
    public double RightServoMiddle = 0.15;
    public double LeftServoMiddle = 0.85;

    public double ArmClose = 0.8;
    public double ArmOpen = 0.3;
    public boolean IntakeStop = true;
    public int PointIndexStartElavator = 0;
    public double PlacingStoneTime = 0;

    public double cubeNotInMM = 150;


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

        cubeIn = hardwareMap.get(DistanceSensor.class, "cubeIn");
        Touch_Foundation = hardwareMap.get(DigitalChannel.class, "Touch_Foundation");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


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
        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);

        IntakeL.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE the intake System

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
        IMU.initialize(parameters);
/*
        // make sure the imu gyro is calibrated before continuing.
        while (!isStarted() && !isStopRequested() && !IMU.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        */
        angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        Velocity = IMU.getVelocity();
        Acceleration = IMU.getAcceleration();


        // Define and initialize ALL installed servos.
        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);
//        Output.setPosition(OutputUp);


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
        double odometryHorizental = (-rightLinearMotor.getCurrentPosition());
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

    public boolean PlacingStoneWhitTime(TelemetryPacket packet) {
        boolean ElvateBusy = true;
        double Delta = runtime.seconds() - PlacingStoneTime;
        packet.put("delte", Delta);
        if (Delta >= 0.3 && Delta <= 1) {
            MyElevator.ElevateWithEncoder(-350, 0.4, 0.3);
        }
        if (Delta >= 1.1 && Delta <= 2) {
            Arm.setPosition(ArmOpen);
        }
        if (Delta >= 2.1 && Delta <= 2.5) {
            MyElevator.ElevateWithEncoder(0, 0.3, 0.0035);
        }
        if (Delta >= 2.6 && Delta <= 3.6) {
            Output.setPosition(OutputOpen);
        }
        if (Delta >= 3.7 && Delta <= 4.7) {
            MyElevator.ElevateWithEncoder(-350, 0.4, 0.3);
        }
        if (Delta >= 4.8 && Delta <= 5.5) {
            Arm.setPosition(ArmClose);
        }
        if (Delta >= 5.6 && Delta <= 6) {
            MyElevator.ElevateWithEncoder(0, 0.4, 0.0035);
            ElvateBusy = false;
        }
        return ElvateBusy;
    }

}

