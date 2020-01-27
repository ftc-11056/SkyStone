package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "GilatTeleop", group = "teleops")
public class GilatTeleop extends LinearOpMode {

//    Level Counter
    ElapsedTime runtime = new ElapsedTime();
    private int stayingPosition = 0;
    private int encodersStay;
    private double time = 0;

    private int stayErrors = 0;
    private double stayPN = 0.01;
    private double power = 0;
    private double stayPower = 0;

    private boolean autoY = false;
    private boolean up = false;
    private boolean low = false;

    private int Level = 200;
    private int counter = 1;
    private int pos = 0;
    public Servo Capstone = null;
    public double CapstoneUp = 0.15  ;
    public double CapstoneDown = 0.54;
    private boolean Capass = false;
    public Servo Arm = null;
    public Servo Output = null;
    public DcMotor leftLinearMotor = null;
    public DcMotor rightLinearMotor = null;

    int rightEncoderPosition = 0;
    int downPOS = 0;
    int upPOS = 0;
    private boolean counterflag = false;
    private boolean endOfY = false;
    private boolean firstRase = false;


    private int stayCounter = 0;
    private double stayDN = 0.00001;

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
    private int anotherDownVar = 0;

    private int fixAuto = 0;
    public double OutputClose = 0.7;
    public double OutputOpen = 0.4;
    public double ArmClose = 0.7;
    public double ArmOpen = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        // super.runOpMode();
        runtime.reset();
////
//        rightLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Arm = hardwareMap.get(Servo.class, "Arm");

        Output = hardwareMap.get(Servo.class, "OutPut");
        leftLinearMotor = hardwareMap.get(DcMotor.class, "leftLinearMotor");
        rightLinearMotor = hardwareMap.get(DcMotor.class, "rightLinearMotor");

        rightLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad2.dpad_up) {
                Output.setPosition(0.4);
            } else if (gamepad2.dpad_down) {
                Output.setPosition(0.7);
            }

            if (gamepad2.x) {
                Arm.setPosition(0.25);
            } else if (gamepad2.b) {
                Arm.setPosition(0.7);

            }


            pos = -counter * Level;

//            TODO: Auto Y
            if (gamepad2.y) {
                autoY = true;
                Output.setPosition(OutputClose);
                time = runtime.seconds();
                telemetry.addLine("1");
            }
            if (autoY == true && (-time + runtime.seconds() > 1.2)) {
                stayingPosition = pos;
                power = 1;
                stayPN = 0.01;
                Arm.setPosition(ArmOpen);
                telemetry.addLine("2");
                autoY = false;
                counter += 1;
            }

//            TODO: Auto A
            if (gamepad2.a){
                Arm.setPosition(ArmClose);
                Output.setPosition(OutputOpen);
                stayingPosition = 0;
                power = 0.3;
                stayPN = 0.005;
            }

//            TODO: One Level Upper
            if (gamepad2.left_stick_y < -0.5 && gamepad2.left_stick_button) {
                up = true;
                stayingPosition = pos;
                power = 1;
                stayPN = 0.01;
            }else if (up && leftLinearMotor.getCurrentPosition() <= pos + 90) {
                up = false;
                counter += 1;
            }

            //      TODO: Reset Counter
            if (gamepad2.dpad_right) counter = 1;

//            TODO: One Level Lower
            if (gamepad2.left_stick_y > 0.5 && gamepad2.left_stick_button) {
                low = true;
                stayingPosition = pos + Level + 10;
                stayPN = 0.005;
            }else if (low && leftLinearMotor.getCurrentPosition() >= pos + 90) {
                low = false;
                counter -= 1;
            }

//            TODO: Capstone Button
/*
            if (gamepad2.right_bumper) {
                time = runtime.seconds();
                Output.setPosition(OutputOpen);
                Capass = true;
            }
            if(Capass&&(-time + runtime.seconds() > 1.2)) {
                stayingPosition = -300;
                power = 0.5;
                stayPN = 0.01;
            }
            if (Capass && leftLinearMotor.getCurrentPosition() <= -280){
                Capstone.setPosition(CapstoneDown);
            }
            if (Capass && (-time + runtime.seconds() > 2.4)){
                stayingPosition = 0;
                power=0.3;
                stayPN=0.001;
            }
            if (Capass && (-time + runtime.seconds() > 3.6) && stayingPosition >= -10) {
                Output.setPosition(OutputClose);
            }
            if (Capass && (-time + runtime.seconds() > 4.8) && stayingPosition >= -10){
                Arm.setPosition(ArmOpen);
                Capass = false;
            }
            /*if (Arm.getPosition() == ArmOpen) Capass = false;
             */
//            if (gamepad1.right_trigger > 0) Capstone.setPosition(0.3);
//            else if (gamepad1.left_trigger > 0) Capstone.setPosition(0.7);

//            TODO: Slow Down

            if (gamepad2.left_bumper) {
//                MyElevator.ElevateWithEncoder(-10, 0.1, 0.0088);
                leftLinearMotor.setTargetPosition(-10);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(0.2);
                rightLinearMotor.setPower(0.2);
                stayingPosition = leftLinearMotor.getCurrentPosition();
            }
//            TODO: the only move command
            else if (leftLinearMotor.getCurrentPosition() > -1450 && leftLinearMotor.getCurrentPosition() <-20 ){
                stayErrors = leftLinearMotor.getCurrentPosition() - stayingPosition;
                stayPower = power * stayErrors * stayPN;
                leftLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(stayPower);
                rightLinearMotor.setPower(stayPower);
                if (leftLinearMotor.getCurrentPosition() == pos) stayPN = 0.001;
            }else if (leftLinearMotor.getCurrentPosition() < -20) {
                leftLinearMotor.setPower(0);
                rightLinearMotor.setPower(0);
                stayingPosition = -20;
            }else {
                leftLinearMotor.setPower(0);
                rightLinearMotor.setPower(0);
                stayingPosition = -1440;
            }

            encodersStay = stayingPosition;

            telemetry.addData("leftElevator:", leftLinearMotor.getCurrentPosition());
            telemetry.addData("autoY", autoY);
            telemetry.addData("counter", counter);
            telemetry.update();

        }
    }

            //        TODO: Levels Counter

/*
            //          TODO: One Level Upper
            if (gamepad2.b && !up) {
                up = true;
                MyElevator.ElevateWithEncoder(pos, 1, 1);
                stayingPosition = pos;
                telemetry.addLine("Elevator move");
            }
            if (up && leftLinearMotor.getCurrentPosition() <= pos + 90 && !gamepad2.b) {
                up = false;
                counter += 1;
            }
//            TODO: One Level Lower

            if (gamepad2.x && !low) {
                low = true;
                MyElevator.ElevateWithEncoder(pos + 200, 0.6, 0.003);
                stayingPosition = pos + 200;
                telemetry.addLine("Elevator move");
            }
            if (low && leftLinearMotor.getCurrentPosition() >= pos - 90 && !gamepad2.x) {
                low = false;
                counter -= 1;
            }

            //      TODO: Reset Counter
            if (gamepad2.dpad_right) counter = 1;


//          TODO:  Elevator:
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
                        MyElevator.ElevateWithEncoder(-500,1,0.005);
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
/*
            encodersStay = stayingPosition;


            telemetry.addData("current Position LeftElevator", leftLinearMotor.getCurrentPosition());
            telemetry.addData("counter", counter);
            telemetry.addData("POS", pos);
            telemetry.addData("right", rightLinearMotor.getPower());
            telemetry.addData("left", leftLinearMotor.getPower());
            telemetry.update();

            //        TODO: Hillel Mechanizem
            if (gamepad1.right_bumper) {
                Arm.setPosition(0.32);
                //  ArmMode = "in";
            } else if (gamepad1.right_trigger > 0) {
                Arm.setPosition(1);
                //   ArmMode = "out";
            }

            if (gamepad1.left_bumper) {
                Output.setPosition(0.63);
                // ArmMode = "in";
            } else if (gamepad1.left_trigger > 0) {
                Output.setPosition(1);
                // ArmMode = "out";
            }

            //     TODO: touch sensor
            if (Touch_Foundation.getState() == false) {
                telemetry.addLine("1 1 0 5 6   T H E  Y  B O T ");

            }
            if (cubeIn.getDistance(DistanceUnit.MM) > cubeNotInMM) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else if (cubeIn.getDistance(DistanceUnit.MM) < cubeNotInMM){
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            }
        }
    }
}*/
        }


