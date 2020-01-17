package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "goBuildaTeleop", group = "teleops")
public class goBuildaTeleop extends Robot {

    //    TODO: values
    private double ledTime = 0;
    private boolean ledColor = true;

    // normal down mode
    private String ArmMode = "in";

    //    Level Counter

    private int Level = 100;
    private int counter = 1;
    private int pos = 0;
    private boolean low = false;
    private boolean up = false;
    private boolean endOfY = false;
    private boolean firstRase = false;

    private int stayingPosition = 0;
    private int encodersStay;
    private double time = 0;
    private int stayErrors = 0;

    private double stayPN = 0.001;
    private double power = 0;

    //    up values:
    private boolean upDegel = false;
    private boolean YDondMove = true;
    private boolean counterbool = false;

    //    down values
    private int downDegel = 0;
    private int downDegelToServo = 0;
    private boolean flag = false;
    private boolean bumpersDondMove = true;
    private boolean ADondMove = true;
    private boolean underMagnet = false;
    private boolean Abutton = false;

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

//           TODO: Servos:
            if (gamepad2.dpad_left) {
                Arm.setPosition(ArmClose);
                ArmMode = "in";
            } else if (gamepad2.dpad_right) {
                Arm.setPosition(ArmOpen);
                ArmMode = "out";
            }

            if (gamepad2.dpad_up) {
                Output.setPosition(OutputUp);
            } else if (gamepad2.dpad_down) {
                Output.setPosition(OutputDown);
            }

            if (gamepad2.x) {
                Arm.setPosition(ArmOpen);
            } else if (gamepad2.b) {
                Arm.setPosition(ArmClose);

            }

            if (gamepad1.left_bumper) {
                ParkingMot.setPosition(ParkingMotIn);
            } else if (gamepad1.right_bumper) {
                ParkingMot.setPosition(ParkingMotOut);
            }

            if (gamepad2.left_stick_y > 0.7 && gamepad2.left_stick_button) {
                Capstone.setPosition(CapstoneUp);
            } else if (gamepad2.left_stick_y < -0.7 && gamepad2.right_stick_button) {
                Capstone.setPosition(CapstoneDown);
            }


//            TODO: Intake System
            if (gamepad2.right_trigger > 0) {
                if (cubeIn.getDistance(DistanceUnit.MM) < cubeNotInMM)
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                else {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    MyIntake.maxIntake();
                }
            } else if (gamepad2.left_trigger > 0) {
                MyIntake.maxOuttake();
            } else {
                MyIntake.ShutDown();
            }

            if (gamepad1.a) {
                LeftServo.setPosition(LeftServoDown);
                RightServo.setPosition(RightServoDown);
            } else if (gamepad1.y) {
                LeftServo.setPosition(LeftServoUp);
                RightServo.setPosition(RightServoUp);
            }

//TODO:[[[[[[[[[[[[[[[[[[[[[ELEVATOR]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

            pos = -counter * Level;

            //          TODO: One Level Uper
            if (gamepad2.left_stick_y > 0.7 && gamepad2.left_stick_button && !up) {
                up = true;
                MyElevator.ElevateWithEncoder(pos, 1, 1);
                stayingPosition = pos;
            }
            if (up && leftLinearMotor.getCurrentPosition() <= pos + 90 && !gamepad2.b) {
                up = false;
                counter += 1;
            }
//            TODO: One Level Lower
            if (gamepad2.left_stick_y < -0.7 && gamepad2.left_stick_button && !low) {
                low = true;
                MyElevator.ElevateWithEncoder(pos + 200, 0.6, 0.003);
                stayingPosition = pos + 200;
            }
            if (low && leftLinearMotor.getCurrentPosition() >= pos - 90 && !gamepad2.x) {
                low = false;
                counter -= 1;
            }

            //      TODO: Reset Counter
            if (gamepad2.right_bumper) counter = 1;

//          TODO: YY Auto Button:

            if (gamepad2.y) {
                YDondMove = false;
                upDegel = true;
                flag = true;
                underMagnet = false;
                counterbool = true;

                if (upDegel == true) {
                    time = runtime.seconds();
                }
                Output.setPosition(OutputDown);
                telemetry.addData("time is:", time);
                telemetry.update();
            } else if (upDegel == true && downDegel != 1) {
                if (((-time + runtime.seconds()) > 0.7 && (-time + runtime.seconds()) < 1.9) && firstRase == false) {
                    if (upDegel == true) {
                        MyElevator.ElevateWithEncoder(-500, 1, 0.005);
                    }
                    stayingPosition = -400;
                    if (leftLinearMotor.getCurrentPosition() < -350) {
                        Arm.setPosition(1);
                    }
                }
                if ((-time + runtime.seconds()) > 1.9 && leftLinearMotor.getCurrentPosition() < -390 && counterbool) {
                    MyElevator.ElevateWithEncoder(pos, 1, 0.01);
                    telemetry.addLine("Elevator move");
                    stayingPosition = pos;
                    counter += 1;
                    counterbool = false;
                    firstRase = true;
                    endOfY = true;
                }
            }

//          TODO: AA Auto Button:
            if (gamepad2.a) {
                Output.setPosition(OutputUp);
                MyElevator.ElevateWithEncoder(-400, 1, 1);
                telemetry.addLine("Elevator move");
                ADondMove = false;
                downDegel = 1;
                flag = true;
                time = runtime.seconds();
            }
            if (leftLinearMotor.getCurrentPosition() < -350 && downDegel == 1) {
                Arm.setPosition(ArmClose);
                rightLinearMotor.setPower(0);
            }
            if (downDegel == 1 && upDegel != true)
                if ((-time + runtime.seconds()) > 1.7) {
                    MyElevator.ElevateWithEncoder(20, 0.3, 0.003);
                    telemetry.addLine("Elevator move");
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                }

//            TODO: reset auto Buttons:
            if (pos >= -400 && endOfY == true) {
                if ((leftLinearMotor.getCurrentPosition() >= (pos - 10) && Arm.getPosition() == ArmOpen)
                        || gamepad2.b || gamepad2.x || gamepad2.a) {
                    telemetry.addLine(" in reset POS >>> -400");
                    upDegel = false;
                    flag = false;
                    YDondMove = true;
                    endOfY = false;
                    firstRase = false;
                }
            }
            if (pos < -400 && endOfY == true) {
                if ((leftLinearMotor.getCurrentPosition() < (pos + 100 + 10) && Arm.getPosition() == ArmOpen)
                        || gamepad2.b || gamepad2.x || gamepad2.a) {
                    telemetry.addLine(" in reset POS <<< -400");
                    upDegel = false;
                    flag = false;
                    YDondMove = true;
                    firstRase = false;
                    endOfY = false;
                }
            }

            if (gamepad2.left_bumper || gamepad2.right_bumper || leftLinearMotor.getCurrentPosition() > 0 ||
                    downMagnetElevator.getState() == false || gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.y) {
                downDegel = 0;
                downDegelToServo = 0;
                flag = false;
                ADondMove = true;
                Abutton = false;

            }

//            TODO: normal moving
            if (gamepad2.right_bumper && leftLinearMotor.getCurrentPosition() > -400) {
                MyElevator.ElevateWithEncoder(-430, 1, 1);
                telemetry.addLine("Elevator move");
                stayingPosition = leftLinearMotor.getCurrentPosition();
                bumpersDondMove = false;
            } else if (gamepad2.left_bumper && downMagnetElevator.getState() == true) {
                MyElevator.ElevateWithEncoder(20, 0.1, 0.0088);
                stayingPosition = leftLinearMotor.getCurrentPosition();
                bumpersDondMove = false;
            }
//            TODO: stop commands
            else if (downMagnetElevator.getState() == true && upDegel == false && downDegel == 0 && downDegelToServo == 0
                    && ADondMove && !gamepad2.a && low == false && up == false) {
                stayErrors = leftLinearMotor.getCurrentPosition() - stayingPosition;
                power = 1 * stayErrors * stayPN;
                leftLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(power);
                rightLinearMotor.setPower(power);
                telemetry.addLine("Stop Elevator");
            } /*else if (upDegel == false && downDegel == 0 && downDegelToServo == 0 && ADondMove && !gamepad2.a) {
                leftLinearMotor.setPower(0);
                rightLinearMotor.setPower(0);
            }*/

            encodersStay = stayingPosition;


//            TODO: telemetryes
            telemetry.addData("current Position LeftElevator", leftLinearMotor.getCurrentPosition());
            telemetry.addData("Left power Elevator", leftLinearMotor.getPower());
            telemetry.addData("right power Elevator", rightLinearMotor.getPower());
            telemetry.addData("down sensor Magnet", downMagnetElevator.getState());
            telemetry.addData("Mode: ", MyDriveTrain.Mode);
            telemetry.update();


        }
    }
}
