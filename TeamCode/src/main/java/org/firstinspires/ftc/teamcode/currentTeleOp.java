package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name = "currentTeleOpTest", group = "teamcode")
public class currentTeleOp extends Robot {

//    stay values:
    private int stayingPosition = 0;
    private int encodersStay;
    private double time = 0;
    private int stayErrors = 0;
    private int stayCounter = 0;

    private double stayPN = 0.001;
    private double stayDN = 0.00001;
    private double power = 0;

//    up values:
    private boolean upDegel = false;
    private boolean YDondMove = true;

//    down values
    private int downDegel = 0;
    private int downDegelToServo = 0;
    private boolean flag = false;
    private boolean bumpersDondMove = true;
    private boolean ADondMove = true;
    private boolean underMagnet = false;
    private boolean Abutton = false;
    private int anotherDownVar = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        runtime.reset();

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
//TODO[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 11111111111]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

//            TODO: Drive
            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = angles.firstAngle;

            if (gamepad1.x) MyDriveTrain.setMode("arcade");
            else if (gamepad1.b) MyDriveTrain.setMode("Oriented");

            if (MyDriveTrain.getMode().equals("Oriented")) {
                MyDriveTrain.fieldOriented(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, heading);
            } else {
                MyDriveTrain.arcade(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

//TODO[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 222222222222222]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

//           TODO: Servos:
            if (gamepad2.dpad_left) {
                Arm.setPosition(0.135);
            } else if (gamepad2.dpad_right) {
                Arm.setPosition(1);
            }

            if (gamepad2.dpad_up) {
                Output.setPosition(0.65);
            } else if (gamepad2.dpad_down) {
                Output.setPosition(0.2);
            }

//            TODO: Intake Train
            if (gamepad2.right_trigger > 0) {
                MyIntake.maxIntake();
            } else if (gamepad2.left_trigger > 0) {
                MyIntake.maxOuttake();
            } else {
                MyIntake.ShutDown();
            }

            if (gamepad1.x) {
                LeftServo.setPosition(0.05);
                RightServo.setPosition(0.55);
            } else if (gamepad1.b) {
                LeftServo.setPosition(0.55);
                RightServo.setPosition(1);
            }

//          TODO:  Elevator:

//          TODO: YY Auto Button:
            if (gamepad2.y) {
                YDondMove = false;
                upDegel = true;
                flag = true;
                underMagnet = false;
                if (upDegel == true) {
                    time = runtime.seconds();
                }
                Output.setPosition(0.2);
                telemetry.addData("time is:", time);
                telemetry.update();
            } else if (upDegel == true && downDegel != 1)
                if ((-time + runtime.seconds()) > 2) {
                    MyElevator.ElevateWithEncoder(-400, 0.3, 0.5);
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                    telemetry.addLine("Here");
                    telemetry.update();
                    if (leftLinearMotor.getCurrentPosition() < -380 || rightLinearMotor.getCurrentPosition() < -380) {
                        Arm.setPosition(1);
                    }
                }
//          TODO: AA Auto Button:
            if (gamepad2.a) {
                Abutton = true;
                ADondMove = false;
                flag = true;
                downDegel = 1;
            }
            if (Abutton == true) {
                if (downDegel == 1 && anotherDownVar == 0) {
                    MyElevator.ElevateWithEncoder(-460, 0.3, 0.5);
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                }
                stayingPosition = leftLinearMotor.getCurrentPosition();

                if (rightLinearMotor.getCurrentPosition() < -400 || leftLinearMotor.getCurrentPosition() < -400) {
                    Arm.setPosition(0.135);

                }
                if (rightLinearMotor.getCurrentPosition() < -450 || leftLinearMotor.getCurrentPosition() < -450) {
                    Output.setPosition(0.65);
                    MyElevator.ElevateWithEncoder(0, 0.1, 0.01);
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                    downDegel = 2;
                    telemetry.addData("time is:", time);
                    telemetry.update();
                    anotherDownVar = 1;
                }
            }else anotherDownVar = 0;

//            TODO: reset auto Buttons:
            if (leftLinearMotor.getCurrentPosition() < -380 || rightLinearMotor.getCurrentPosition() < -380) {
                upDegel = false;
                flag = false;
                YDondMove = true;
            }
            if (gamepad2.left_bumper || gamepad2.right_bumper || leftLinearMotor.getCurrentPosition() > -0 || rightLinearMotor.getCurrentPosition() > -0
                    || downMagnetElevator.getState() == false) {
                downDegel = 0;
                downDegelToServo = 0;
                flag = false;
                ADondMove = true;
                Abutton = false;
            }

//            TODO: normal moving
            if (gamepad2.right_bumper && leftLinearMotor.getCurrentPosition() > -380 /*&& upMagnetElevator.getState() == false*/) {
                MyElevator.ElevateWithEncoder(-400, 0.23, 0.5);
                stayingPosition = leftLinearMotor.getCurrentPosition();
                bumpersDondMove = false;
                underMagnet = false;
            } else if (gamepad2.left_bumper && downMagnetElevator.getState() == true) {
                MyElevator.ElevateWithEncoder(0, 0.15, 0.01);
                stayingPosition = leftLinearMotor.getCurrentPosition();
                bumpersDondMove = false;
            }
//            TODO: stop commands
            else if (downMagnetElevator.getState() == true && upDegel == false && downDegel == 0 && downDegelToServo == 0 && ADondMove && !gamepad2.a) {
                stayErrors = leftLinearMotor.getCurrentPosition() - stayingPosition;
//                stayCounter = stayCounter + leftLinearMotor.getCurrentPosition(+ );
                power = 1 * stayErrors * stayPN /*+ stayCounter * stayDN*/;
                leftLinearMotor.setTargetPosition(encodersStay);
                rightLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(power);
                rightLinearMotor.setPower(power);
            } else if (upDegel == false && downDegel == 0 && downDegelToServo == 0 && ADondMove && !gamepad2.a) {
                leftLinearMotor.setPower(0);
                rightLinearMotor.setPower(0);
            }

            encodersStay = stayingPosition;

//            TODO: telemetryes
            telemetry.addData("current Position LeftElevator", leftLinearMotor.getCurrentPosition());
            telemetry.addData("Left power Elevator", leftLinearMotor.getPower());
            telemetry.addData("right power Elevator", rightLinearMotor.getPower());
            telemetry.addData("Value under Magnet", underMagnet);
            telemetry.addData("down sensor Magnet", downMagnetElevator.getState());
            telemetry.addData("downDegel", downDegel);
            telemetry.addData("Mode: ", MyDriveTrain.Mode);
            telemetry.addData("stay D n: ", stayDN);
            telemetry.update();

        }
    }
}
