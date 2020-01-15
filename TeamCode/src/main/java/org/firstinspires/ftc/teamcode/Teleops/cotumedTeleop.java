package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "cotumedTeleop", group = "teleops")
public class cotumedTeleop extends Robot {

    private int stayingPosition = 0;
    private int encodersStay;
    private double time = 0;
    private int stayErrors = 0;
    private double stayPN = 0.001;
    private double power = 0;
    private double stayPower = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        runtime.reset();

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        blinkinLedDriver.setPattern(pattern);

        waitForStart();
        while (opModeIsActive()){
            //TODO[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 11111111111]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

//            TODO: Drive
            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = angles.firstAngle;

            if (gamepad1.x) MyDriveTrain.setMode("arcade");
            else if (gamepad1.b) MyDriveTrain.setMode("Oriented");

            if (MyDriveTrain.getMode().equals("Oriented")) {
                MyDriveTrain.fieldOriented(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, heading + -360);
            } else {
                MyDriveTrain.arcade(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            if (gamepad1.dpad_up) {
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
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

//TODO[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 222222222222222]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

            if (gamepad2.y){
                stayingPosition = -1000;
                power = 1;
            }else if (gamepad2.b){
                stayingPosition = -700;
                power = 0.8;
            }else if (gamepad2.a){
                stayingPosition = -300;
                power = 0.5;
            }else if (gamepad2.x){
                stayingPosition = 0;
                power = 0.2;
            } else {
                stayErrors = leftLinearMotor.getCurrentPosition() - stayingPosition;
                stayPower = power * stayErrors * stayPN;
                leftLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(stayPower);
                rightLinearMotor.setPower(stayPower);
                telemetry.addLine("Stop Elevator");
            }

            encodersStay = stayingPosition;

            telemetry.addData("leftElevator:", leftLinearMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}

