package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name = "TeleOpTest", group = "teamcode")
public class TeleOpTest extends Robot {

    private int stayingPosition = 0;
    private int encodersStay;
    private double time = 0;
    private boolean upDegel = false;
    private boolean downDegel = false;
    private boolean flag = false;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        runtime.reset();

        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 11111111111]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = angles.firstAngle;

            if (gamepad1.x) MyDriveTrain.setMode("arcade");
            else if (gamepad1.b) MyDriveTrain.setMode("Oriented");

            telemetry.addData("Mode: ", MyDriveTrain.Mode);
            telemetry.update();

            if (MyDriveTrain.getMode().equals("Oriented")) {
                MyDriveTrain.fieldOriented(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, heading);
            } else {
                MyDriveTrain.arcade(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            if(gamepad1.right_trigger>0){
                LB.setPower(1);
                LF.setPower(1);
            }
//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 222222222222222]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]`

//            Servos:

            if (gamepad2.dpad_up) {
                Arm.setPosition(0);
            } else if (gamepad2.dpad_down) {
                Arm.setPosition(1);
            }

            if (gamepad2.dpad_right) {
                Output.setPosition(1);
            } else if (gamepad2.dpad_left) {
                Output.setPosition(0);
            }

            if (gamepad2.right_trigger > 0) {
                MyIntake.maxIntake();
            } else if (gamepad2.left_trigger > 0) {
                MyIntake.maxOuttake();
            } else {
                MyIntake.ShutDown();
            }

//            Elevator:
            //            Auto Button:*
            if (gamepad2.y) {
                upDegel = true;
                flag = true;
                if (upDegel == true) {
                    time = runtime.seconds();
                }
                Output.setPosition(0.75);
                telemetry.addData("time is:", time);
                //sleep(2000);
                telemetry.update();
            } else if (upDegel == true)
                if ((-time + runtime.seconds()) > 2) {
                    MyElevator.ElevateWithEncoder(-400, 0.3, 0.5);
                    /*if (rightLinearMotor.getCurrentPosition()>300 || leftLinearMotor.getCurrentPosition()>300)
                        MyElevator.dontMoveElevator(1,300);*/
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                    telemetry.addLine("Here");
                    telemetry.update();
                    if (leftLinearMotor.getCurrentPosition() < -380 || rightLinearMotor.getCurrentPosition() < -380) {
                        Arm.setPosition(0.75);
                    }
                }
            if (leftLinearMotor.getCurrentPosition() < -380 || rightLinearMotor.getCurrentPosition() < -380) {
                upDegel = false;
                flag = false;
            }

            if (gamepad2.a) {
                downDegel = true;
                flag = true;
                if (downDegel == true) {
                    time = runtime.seconds();
                }
                Arm.setPosition(0.25);
                telemetry.addData("time is:", time);
                //sleep(2000);
                telemetry.update();
            } else if (downDegel == true)
                if ((-time + runtime.seconds()) > 2) {
                    MyElevator.ElevateWithEncoder(-50, 0.3, 0.002);
                    /*if (rightLinearMotor.getCurrentPosition()>300 || leftLinearMotor.getCurrentPosition()>300)
                        MyElevator.dontMoveElevator(1,300);*/
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                    telemetry.addLine("Here");
                    telemetry.update();
                    if (leftLinearMotor.getCurrentPosition() > -60 || rightLinearMotor.getCurrentPosition() > -60) {
                        Output.setPosition(0.25);
                    }
                }
            if (leftLinearMotor.getCurrentPosition() > -60 || rightLinearMotor.getCurrentPosition() > -60) {
                downDegel = false;
                flag = false;
            }

            if (gamepad2.right_bumper && leftLinearMotor.getCurrentPosition() > -380 /*&& upMagnetElevator.getState() == false*/) {
                MyElevator.ElevateWithEncoder(-400, 0.3, 0.5);
                stayingPosition = leftLinearMotor.getCurrentPosition();
            } else if (gamepad2.left_bumper && leftLinearMotor.getCurrentPosition() < -50/*&& downMagnetElevator.getState() == false*/) {
                MyElevator.ElevateWithEncoder(-50, 0.3, 0.002);
                stayingPosition = leftLinearMotor.getCurrentPosition();
            } /*else if (flag == false) {
                leftLinearMotor.setTargetPosition(encodersStay);
                rightLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(1);
                rightLinearMotor.setPower(1);
            }*/

            encodersStay = stayingPosition;

            if (gamepad2.x) {
                LeftServo.setPosition(0);
                RightServo.setPosition(0.25);
            } else if (gamepad2.b) {
                LeftServo.setPosition(0.6);
                RightServo.setPosition(0.9);
            }


            telemetry.addData("stay values", stayingPosition);
            telemetry.addData("motor encoders", leftLinearMotor.getCurrentPosition());
            telemetry.addData("down Degel", downDegel);
            telemetry.addData("up Degel", upDegel);
            telemetry.addData("flag", flag);
            telemetry.update();

        }
    }
}














