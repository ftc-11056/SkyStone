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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotCustomade;

@TeleOp(name = "CustuMadeTeleop", group = "teleops")
public class CustuMadeTeleop extends RobotCustomade {

    private int stayingPosition = 0;
    private int encodersStay;
    private double time = 0;

    private int stayErrors = 0;
    private double stayPN = 0.01;
    private double power = 0;
    private double stayPower = 0;

    private boolean autoY = false;
    private boolean autoA = false;
    private boolean autoAUpEle = false;
    private boolean autoAArm = false;
    private boolean autoYDownEle = false;
    private boolean autoAB = false;
    private boolean autoAUpEleB = false;
    private boolean up = false;
    private boolean low = false;

    private int Level = 220;
    private int counter = 1;
    private int pos = 0;

    private boolean Capass = false;
    private boolean slowDown = false;
    private boolean normalUp = false;

    private double leftStickX = 0;
    private double leftStickY = 0;
    private double rightStickX = 0;
    private int foundation= 0;
    private int katze= 0;
    private double Armposin=0;
    private double Armposout=0;
    private double Armpos=0;

    private boolean CapstoneElevator = false;

    private boolean moveOff = false;

//    new elevator move

    private boolean lowerLimit = true;
    private boolean upperLimit = true;
    private boolean magnet = true;

    private boolean resetStart = true;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        runtime.reset();

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLinearMotor.setDirection(DcMotor.Direction.REVERSE);

        pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        blinkinLedDriver.setPattern(pattern);

        while (!isStarted()){
            runtime.reset();
        }
        waitForStart();
        while (opModeIsActive()) {
            if (runtime.seconds() < 0.5){
                stayingPosition = -50;
                power = 0.05;
                stayPN = 0.06;
                telemetry.addLine("Elevator isnt work");
            }else if (resetStart){
                resetStart = false;
                leftLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                stayingPosition = 0;
            }
            //TODO[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 11111111111]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

//            TODO: Drive
            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            double heading = angles.firstAngle;

            if (gamepad1.x) MyDriveTrain.setMode("arcade");
            else if (gamepad1.b) MyDriveTrain.setMode("Oriented");

            if (MyDriveTrain.getMode().equals("Oriented")) {
                MyDriveTrain.fieldOriented(-leftStickY, leftStickX, rightStickX, -heading);
            } else {
                MyDriveTrain.arcade(-leftStickY, leftStickX, rightStickX);
            }

            if (gamepad1.right_trigger > 0) {
                leftStickX = gamepad1.left_stick_x * 0.5;
                leftStickY = gamepad1.left_stick_y * 0.5;
                rightStickX = gamepad1.right_stick_x * 0.5;
            } else {
                leftStickX = gamepad1.left_stick_x;
                leftStickY = gamepad1.left_stick_y;
                rightStickX = gamepad1.right_stick_x;
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

            if (gamepad1.y) {
                LeftServo.setPosition(LeftServoUp);
                RightServo.setPosition(RightServoUp);

            } else if (gamepad1.a) {
                LeftServo.setPosition(LeftServoDown);
                RightServo.setPosition(RightServoDown);
            }

            if (gamepad1.left_bumper) {
                ParkingMot.setPosition(ParkingMotIn);
            } else if (gamepad1.right_bumper) {
                ParkingMot.setPosition(ParkingMotOut);
            }


            /*if (gamepad1.right_bumper){
                Arm.setPosition(ArmClose-Armposin);
                Armposin+=0.00001;
            }
            else if (gamepad1.left_bumper){
                Arm.setPosition(ArmOpen+Armposout);
                Armposout+=0.00003;
            }*/

//TODO[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 222222222222222]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

//           TODO: Servos:
            if (gamepad2.dpad_up) {
                Output.setPosition(OutputOpen);
            } else if (gamepad2.dpad_down) {
                Output.setPosition(OutputClose);
            }

           /* if (gamepad2.x) {
                Arm.setPosition(ArmOpen);
            } else if (gamepad2.b) {
                Arm.setPosition(ArmClose);

            }*/
           if (gamepad2.x) {
               Arm.setPosition(ArmOpen);

           } else if
           (gamepad2.b) {
               Arm.setPosition(ArmClose);
           }

         /*   if (gamepad2.left_stick_y > 0.7 && gamepad2.left_stick_button){
                Capstone.setPosition(CapstoneUp);
            }else if (gamepad2.left_stick_y < -0.7 && gamepad2.right_stick_button){
                Capstone.setPosition(CapstoneDown);
            }
*/
//            TODO: Intake System
            if (gamepad2.right_trigger > 0) {
                MyIntake.maxIntake();
            } else if (gamepad2.left_trigger > 0) {
                MyIntake.maxOuttake();
            } else {
                MyIntake.ShutDown();
            }

            /*if (cubeIn.getDistance(DistanceUnit.MM) > cubeNotInMM && gamepad2.right_trigger > 0) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
            } else */if (cubeIn.getDistance(DistanceUnit.MM) < cubeNotInMM && gamepad2.right_trigger > 0) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
            }

            if (cubeIn.getDistance(DistanceUnit.MM) < cubeNotInMM && gamepad2.right_trigger > 0)
                Output.setPosition(OutputClose);

//TODO[[[[[[[[[[[[[[[[[[[[[[[[[[[[[Elevator]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

            if (gamepad2.right_trigger == 0) {
                if (counter == 0 && (leftLinearMotor.getCurrentPosition() > -20))
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
                if (counter == 1 && leftLinearMotor.getCurrentPosition() > -20)
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                else if (counter == 2 && leftLinearMotor.getCurrentPosition() > -20)
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                else if (counter == 3 && leftLinearMotor.getCurrentPosition() > -20)
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                else if (counter == 4 && leftLinearMotor.getCurrentPosition() > -20)
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                else if (counter == 5 && leftLinearMotor.getCurrentPosition() > -20)
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                else if (counter == 6 && leftLinearMotor.getCurrentPosition() > -20)
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                else if (counter == 7 && leftLinearMotor.getCurrentPosition() > -20)
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                else if (counter == 8 && leftLinearMotor.getCurrentPosition() > -20)
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                else if (counter == 9 && leftLinearMotor.getCurrentPosition() > -20)
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }
            if (counter == 1) pos = -90 -counter * Level;
            else if (counter == 2) pos = -30 -counter * Level;
            else if (counter == 3) pos = -15 -counter * Level;
            else if (counter == 4) pos = -35 -counter * Level;
            else pos = -counter * Level;

//            TODO: Auto Y
            if (gamepad2.y) {
                autoY = true;
                time = runtime.seconds();
                telemetry.addLine("1");
                if (counter == 0) stayingPosition = 0;
                else stayingPosition = pos;
                power = 1;
                stayPN = 0.015;
                telemetry.addLine("2");
            }
            if ((autoY == true && leftLinearMotor.getCurrentPosition() <= stayingPosition || rightLinearMotor.getCurrentPosition() <= stayingPosition && stayingPosition == pos && counter !=0)
                    || (autoY == true && stayingPosition == pos && counter == 0)){
                autoY = false;
                telemetry.addLine("3");
            }

//            TODO: Auto left trriger
            if (gamepad1.left_trigger > 0){
                autoYDownEle = true;
                stayingPosition = pos + 200;
                power = 0;
                stayPN = 0.003;
                time = runtime.seconds();
            }else if (autoYDownEle == true){
                Output.setPosition(OutputOpen);
                time = runtime.seconds();
                autoYDownEle = false;
                autoAUpEle = true;
            }if (autoAUpEle && (runtime.seconds() - time > 0.6)) {
                stayingPosition = pos;
                power = 1;
                stayPN = 0.05;
                time = runtime.seconds();
                autoAUpEle = false;
                autoAArm = true;
            }if (autoAArm == true && (runtime.seconds() - time > 0.3)){
                autoAArm = false;
                autoA = true;
                Arm.setPosition(ArmClose);
                time = runtime.seconds();
                telemetry.addLine("1");
            }if (autoA == true && (runtime.seconds() - time > 0.5)){
                Output.setPosition(OutputOpen);
                stayingPosition = 0;
                power = 0;
                stayPN = 0.02;
                telemetry.addLine("2");
                counter += 1;
                autoA = false;
            }

//            TODO: autoA
            if (gamepad2.a) {
                stayingPosition = pos + 50;
                autoAUpEleB = true;
                time = runtime.seconds();
            }if (autoAUpEleB == true && (runtime.seconds() - time > 0.5)){
                autoAUpEleB = false;
                autoA = true;
                Arm.setPosition(ArmClose);
                time = runtime.seconds();
                telemetry.addLine("1");
            }if (autoAB == true && (runtime.seconds() - time > 1)){
                Output.setPosition(OutputOpen);
                stayingPosition = 0;
                power = 0;
                stayPN = 0.005;
                telemetry.addLine("2");
                counter += 1;
                autoAB = false;
            }

//            TODO: One Level Upper
            if (gamepad2.left_stick_y < -0.3 && gamepad2.left_stick_button) {
                up = true;
                stayingPosition = pos;
                power = 1;
                stayPN = 0.01;
            } else if (up && leftLinearMotor.getCurrentPosition() <= pos + 90 && rightLinearMotor.getCurrentPosition() <= pos + 90) {
                up = false;
                counter += 1;
            }

            if (gamepad2.right_stick_y > 0.7){
                leftLinearMotor.setPower(-0.3);
                rightLinearMotor.setPower(-0.3);
            }else if (gamepad2.right_stick_y < -0.7){
                leftLinearMotor.setPower(0.3);
                rightLinearMotor.setPower(0.3);
            }

            //      TODO: Reset Counter
            if (gamepad2.dpad_right) counter = 0;

//            TODO: One Level Lower
            if (gamepad2.left_stick_y > 0.3 && gamepad2.left_stick_button) {
                low = true;
                stayingPosition = pos + Level;
                stayPN = 0.005;
            } else if (low && leftLinearMotor.getCurrentPosition() >= pos + 90 && rightLinearMotor.getCurrentPosition() >= pos + 90) {
                low = false;
                counter -= 1;
            }

//            TODO: Capstone Button
            if (gamepad2.right_stick_button) {
                time = runtime.seconds();
                Output.setPosition(OutputOpen);
                Capass = true;
            }
            if (Capass && (-time + runtime.seconds() > 0.5) && (-time + runtime.seconds() < 1)) {
                stayingPosition = -600;
                power = 1;
                stayPN = 0.015;
            }
            if (Capass && leftLinearMotor.getCurrentPosition() <= -280 || Capass && rightLinearMotor.getCurrentPosition() <= -280) {
                Capstone.setPosition(CapstoneDown);
                CapstoneElevator = true;
            }
            if (CapstoneElevator && Capass && (-time + runtime.seconds() > 2)) {
                stayingPosition = 0;
                power = 0;
                stayPN = 0.005;
                CapstoneElevator = false;
            }
            if (Capass && (-time + runtime.seconds() > 3.2) && stayingPosition >= -10) {
                Output.setPosition(OutputClose);
                CapstoneElevator = false;
            }
            if (Capass && (-time + runtime.seconds() > 3.9) && stayingPosition >= -10) {
                Arm.setPosition(ArmOpen);
                CapstoneElevator = false;
            }
            if (Capass && (-time + runtime.seconds() > 4.5) && stayingPosition >= -10) {
                Capstone.setPosition(CapstoneUp);
                CapstoneElevator = false;
                Capass = false;
            }

            if (gamepad2.y || gamepad2.right_stick_y < -0.5 && gamepad2.left_stick_button){
                Capass = false;
                Capstone.setPosition(CapstoneUp);
            }
//            TODO: normal moving
            if (gamepad2.left_bumper) {
                stayingPosition = -10;
                power = 0;
                stayPN = 0.006;
                slowDown = true;
            } else if (slowDown) {
                stayingPosition = leftLinearMotor.getCurrentPosition();
                slowDown = false;
            }

            if (gamepad2.right_bumper) {
                stayingPosition = -1500;
                power = 0.2;
                stayPN = 0.005;
                normalUp = true;
            } else if (normalUp) {
                stayingPosition = leftLinearMotor.getCurrentPosition();
                normalUp = false;
            }

//            TODO: the only move command
//            Magnet defence
            if (downMagnetElevator.getState() == true || (downMagnetElevator.getState() == false && encodersStay <= 0)) magnet = true;
            else magnet = false;

//            Lower limit defence
            if (encodersStay <= 0) lowerLimit = true;
            else lowerLimit = false;

//            Upper limit defence
            if (encodersStay >= -1600) upperLimit = true;
            else upperLimit = false;

//            The range of the staying position
            if (downMagnetElevator.getState() == false) {
                leftLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (magnet == true && lowerLimit == true & upperLimit == true){
                stayErrors = leftLinearMotor.getCurrentPosition() - encodersStay;
                stayPower = power * stayErrors * stayPN;
                leftLinearMotor.setTargetPosition(encodersStay);
                rightLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(stayPower);
                rightLinearMotor.setPower(stayPower);
                if (stayingPosition <= -1550) encodersStay = -1550;
                else if (stayingPosition >= 0) encodersStay = 0;
                else encodersStay = stayingPosition;

                if (leftLinearMotor.getCurrentPosition() == encodersStay || rightLinearMotor.getCurrentPosition() == encodersStay){ stayPN = 0.001;}

            }else {
                leftLinearMotor.setPower(0);
                rightLinearMotor.setPower(0);
            }


//            TODO: telemetries
            telemetry.addData("rightPower:", rightLinearMotor.getPower());
            telemetry.addData("encodersStay:", encodersStay);
            telemetry.addData("stayingPosition", stayingPosition);
            telemetry.addData("counter", counter);
            telemetry.addData("leftElevator:", leftLinearMotor.getCurrentPosition());
            telemetry.addData("rightElevator:", rightLinearMotor.getCurrentPosition());
            telemetry.addData("downMagnet", downMagnetElevator.getState());
            telemetry.update();

        }
    }
}