package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name = "TeleOpTest", group = "teamcode")
public class TeleOpTest extends Robot {

    private int grandFlag = 0;

    private boolean upStep1 = true;
    private boolean upStep2;
    private boolean upStep3;
    private boolean upStep4;

    private int num = 0;
    private int stayingPosition = 0;
    private double time = 0;
    int degel = 0;
    int flag = 0;


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

//            telemetry.addData("distance", cubeIn.getDistance(DistanceUnit.MM));
//            telemetry.update();

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

            /*if (gamepad1.x) {
                LF.setPower(1);
                LB.setPower(1);
                RF.setPower(1);
                RB.setPower(1);
            }*/

//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[GAMEPAD 222222222222222]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]

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

            if (gamepad1.right_bumper) {
                MyIntake.maxIntake();
            } else if (gamepad1.left_bumper) {
                MyIntake.maxOuttake();
            } else {
                MyIntake.ShutDown();
            }

//            Elevator:

            if (gamepad1.dpad_up && leftLinearMotor.getCurrentPosition() > -420 /*&& upMagnetElevator.getState() == false*/) {
                MyElevator.ElevateWithEncoder(-400, 1, 0.5);
                stayingPosition = leftLinearMotor.getCurrentPosition();
            } else if (gamepad1.dpad_down && leftLinearMotor.getCurrentPosition() < -50/*&& downMagnetElevator.getState() == false*/) {
                MyElevator.ElevateWithEncoder(-50, 0.4, 0.002);
                stayingPosition = leftLinearMotor.getCurrentPosition();
            } else if(flag != 1){
                leftLinearMotor.setTargetPosition(stayingPosition);
                rightLinearMotor.setTargetPosition(stayingPosition);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(1);
                rightLinearMotor.setPower(1);
            }

            stayingPosition = stayingPosition;

//            Auto Button:*

            if (gamepad2.y) {
                degel = 1;
                flag = 1;
                if (degel == 1) {
                    time = runtime.seconds();
                }
                Output.setPosition(0.75);
                telemetry.addData("time is:", time);
                //sleep(2000);
                telemetry.update();
            }
            if (degel == 1)
                if ((-time + runtime.seconds()) > 2) {
                    MyElevator.ElevateWithEncoder(-1000, 1, 0.5);
                    /*if (rightLinearMotor.getCurrentPosition()>300 || leftLinearMotor.getCurrentPosition()>300)
                        MyElevator.dontMoveElevator(1,300);*/
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                    telemetry.addLine("Here");
                    telemetry.update();
                    if (leftLinearMotor.getCurrentPosition() < -980 || rightLinearMotor.getCurrentPosition() < -980) {
                        Arm.setPosition(0.75);
                    }
                }
            if (leftLinearMotor.getCurrentPosition() < -980 || rightLinearMotor.getCurrentPosition() < -980) {
                degel = 0;
                flag = 0;
            }


            if (gamepad2.a) {
                degel = 2;
//                flag = 1;
                if (degel == 2) {
                    time = runtime.seconds();
                }
                Arm.setPosition(0.2);
                telemetry.addData("time is:", time);
                telemetry.update();
            }
            if (degel == 2)
                if ((-time + runtime.seconds()) > 2) {
                    MyElevator.ElevateWithEncoder(200, 1, 1);
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                    Output.setPosition(0);
                    telemetry.addLine("Here");
                    telemetry.update();
                }
            /*if (leftLinearMotor.getCurrentPosition() > -30){
                degel = 0;
                flag = 0;
            }*/

            if (gamepad2.right_trigger > 0) {
                LeftServo.setPosition(0);
                RightServo.setPosition(0.25);
            } else if (gamepad2.left_trigger > 0) {
                LeftServo.setPosition(0.6);
                RightServo.setPosition(0.9);
            }
        }
    }
}












