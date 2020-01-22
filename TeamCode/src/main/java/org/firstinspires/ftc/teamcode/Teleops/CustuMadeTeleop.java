package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotCustomade;

@TeleOp(name = "CustuMadeTeleop", group = "teleops")
public class CustuMadeTeleop extends RobotCustomade {

    private int stayingPosition = 0;
    private int encodersStay;
    private double time = 0;
    private int stayErrors = 0;
    private double stayPN = 0.1;
    private double power = 0;
    private double stayPower = 0;

    private boolean autoY = false;
    private boolean up = false;
    private boolean low = false;


    private int Level = 200;
    private int counter = 1;
    private int pos = 0;

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
        while (opModeIsActive()) {
            //TODO[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 11111111111]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

//            TODO: Drive
            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = angles.firstAngle;

            if (gamepad1.x) MyDriveTrain.setMode("arcade");
            else if (gamepad1.b) MyDriveTrain.setMode("Oriented");

            if (MyDriveTrain.getMode().equals("Oriented")) {
                MyDriveTrain.fieldOriented(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, heading);
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

            if (gamepad1.a) {
                LeftServo.setPosition(LeftServoDown);
                RightServo.setPosition(RightServoDown);
            } else if (gamepad1.y) {
                LeftServo.setPosition(LeftServoUp);
                RightServo.setPosition(RightServoUp);
            }

//TODO[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 222222222222222]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

//           TODO: Servos:
            if (gamepad2.dpad_up) {
                Output.setPosition(OutputUp);
            } else if (gamepad2.dpad_down) {
                Output.setPosition(OutputDown);
            }

            if (gamepad2.x) {
                Arm.setPosition(ArmOpen);
            }
            else if (gamepad2.b){
                Arm.setPosition(ArmClose);

            }

            if (gamepad1.left_bumper) {
                ParkingMot.setPosition(ParkingMotIn);
            }
            else if (gamepad1.right_bumper) {
                ParkingMot.setPosition(ParkingMotOut);
            }

            if (gamepad2.left_stick_y > 0.7 && gamepad2.left_stick_button){
                Capstone.setPosition(CapstoneUp);
            }else if (gamepad2.left_stick_y < -0.7 && gamepad2.right_stick_button){
                Capstone.setPosition(CapstoneDown);
            }


//            TODO: Intake System
            if (gamepad2.right_trigger > 0) {
                MyIntake.maxIntake();
            } else if (gamepad2.left_trigger > 0) {
                MyIntake.maxOuttake();
            } else {
                MyIntake.ShutDown();
            }

//TODO[[[[[[[[[[[[[[[[[[[[[[[[[[[[[Elevator]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
            pos = -counter * Level;

//            TODO: Auto Y
            if (gamepad2.y) {
                autoY = true;
                Output.setPosition(OutputDown);
                time = runtime.seconds();
                telemetry.addLine("1");
            }
            if (autoY == true && (-time + runtime.seconds() > 1.2)) {
                stayingPosition = pos;
                power = 1;
                Arm.setPosition(ArmOpen);
                telemetry.addLine("2");
                autoY = false;
                counter += 1;
            }

//            TODO: Auto A
            if (gamepad2.a){
                stayingPosition = 0;
                Arm.setPosition(ArmClose);
                Output.setPosition(OutputUp);
                power = 0.5;
            }

//            TODO: One Level Upper
            if (gamepad2.left_stick_y < -0.5 && gamepad2.left_stick_button) {
                up = true;
                stayingPosition = pos;
                power = 1;
            }else if (up && leftLinearMotor.getCurrentPosition() <= pos + 90) {
                up = false;
                counter += 1;
            }

            //      TODO: Reset Counter
            if (gamepad2.dpad_right) counter = 1;

//            TODO: One Level Lower
            if (gamepad2.left_stick_y > 0.5 && gamepad2.left_stick_button) {
                low = true;
                stayingPosition = pos + 100 + 10;
            }else if (low && leftLinearMotor.getCurrentPosition() >= pos + 90) {
                low = false;
                counter -= 1;
            }

//            TODO: Slow Downing
            if (gamepad2.left_bumper) {
                MyElevator.ElevateWithEncoder(20, 0.1, 0.0088);
                MyElevator.setStayValues(leftLinearMotor.getCurrentPosition(), 0.2);
            }
//            TODO: the only move command
            else {
                stayErrors = leftLinearMotor.getCurrentPosition() - stayingPosition;
                stayPower = power * stayErrors * stayPN;
                leftLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(stayPower);
                rightLinearMotor.setPower(stayPower);
            }

            encodersStay = stayingPosition;

            telemetry.addData("leftElevator:", leftLinearMotor.getCurrentPosition());
            telemetry.addData("autoY", autoY);
            telemetry.addData("counter", counter);
            telemetry.update();

        }
    }
}