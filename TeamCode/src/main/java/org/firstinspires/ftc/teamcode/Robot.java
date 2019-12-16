package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Systems.DriveTrain;
import org.firstinspires.ftc.teamcode.Systems.IntakeTrain;
import org.firstinspires.ftc.teamcode.Systems.Odometry;
import org.firstinspires.ftc.teamcode.Systems.elevator;

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
    public Servo ParkingMot = null;
    public Servo Capstone = null;

    /*IMU Fileds*/
    protected BNO055IMU IMU = null;
    protected Orientation angles = null;

    //digitalChannels
    public DigitalChannel downMagnetElevator = null;
    public DigitalChannel upMagnetElevator = null;

    /*Analog Sensor*/
    protected DistanceSensor cubeIn;

    /*Mechanisms*/
    protected DriveTrain MyDriveTrain = null;
    protected Odometry MyOdometry = null;
    protected IntakeTrain MyIntake = null;
    protected elevator MyElevator = null;

    public double servoPosition = 0.005;
    protected int fixedPosition = 0;

    public double OutputDown = 0.4;
    public double OutputUp = 0.8;

    public double CapstoneUp = 0.6;
    public double CapstoneDown = 0;


    public double ParkingMotIn = 0.7;
    public double ParkingMotOut = 0;

    public double LeftServoDown = 0.25;
    public double RightServoDown = 0.1;
    public double LeftServoUp = 0.8;
    public double RightServoUp = 0.64;

    public double ArmClose = 0.135;
    public double ArmOpen = 1;

    public double cubeNotInMM = 150;

    public String passWord = "dont pass";

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;

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

        ParkingMot = hardwareMap.get(Servo.class,"ParkingMot");

        Capstone = hardwareMap.get(Servo.class,"Capstone");

        LeftServo  = hardwareMap.get(Servo.class, "LeftServo");
        RightServo  = hardwareMap.get(Servo.class, "RightServo");

        upMagnetElevator = hardwareMap.get(DigitalChannel.class,"upMagnetELevator");
        downMagnetElevator = hardwareMap.get(DigitalChannel.class,"downMagnetELevator");

        cubeIn = hardwareMap.get(DistanceSensor.class, "cubeIn");

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
        IntakeL.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE the intake System



        leftLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to zero power
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
      //  linear_motor.setPower(0);

        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);
        Output.setPosition(OutputUp);
//        Arm.setPosition(0.135);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*Define and Initialize Of IMU*/
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        // Define and initialize ALL installed servos.

        // Define Mechanisms:.
        MyDriveTrain = new DriveTrain(LB,LF, RF, RB, IMU);
        MyIntake = new IntakeTrain(IntakeL, IntakeR);
        MyElevator = new elevator(leftLinearMotor, rightLinearMotor, upMagnetElevator, downMagnetElevator, fixedPosition);
    }
}
