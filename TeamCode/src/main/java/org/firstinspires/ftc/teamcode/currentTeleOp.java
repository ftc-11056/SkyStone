package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name = "currentTeleOpTest", group = "teamcode")
public class currentTeleOp extends Robot {

    private int stayingPosition = 0;
    private int encodersStay;
    private double time = 0;

    private boolean upDegel = false;
    private boolean downDegel = false;
    private boolean flag = false;
    private boolean bumpersDondMove = true;
    private boolean YDondMove = true;
    private boolean ADondMove = true;
    private boolean underMagnet = false;
//    private boolean levels = true;
//    private boolean level1 = false;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        runtime.reset();

//        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

            telemetry.addData("Mode: ", MyDriveTrain.Mode);
            telemetry.update();

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
                Output.setPosition(0.27);
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
                RightServo.setPosition(0.05);
            } else if (gamepad1.b) {
                LeftServo.setPosition(0.55);
                RightServo.setPosition(0.6);
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
                Output.setPosition(0.27);
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
                        Arm.setPosition(1);
                    }
                }
//          TODO: AA Auto Button:
            if (gamepad2.a) {
                ADondMove = false;
                downDegel = true;
                flag = true;
                if (downDegel == true) {
                    time = runtime.seconds();
                }
                Arm.setPosition(0.135);
                telemetry.addData("time is:", time);
                //sleep(2000);
                telemetry.update();
            } else if (downDegel == true)
                if ((-time + runtime.seconds()) > 2) {
                    MyElevator.ElevateWithEncoder(10, 0.15, 0.01);
                    /*if (rightLinearMotor.getCurrentPosition()>300 || leftLinearMotor.getCurrentPosition()>300)
                        MyElevator.dontMoveElevator(1,300);*/
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                    Output.setPosition(0.65);
                    telemetry.addLine("Here");
                    telemetry.update();
                }

            if (leftLinearMotor.getCurrentPosition() < -380 || rightLinearMotor.getCurrentPosition() < -380) {
                upDegel = false;
                flag = false;
                YDondMove = true;
                MyElevator.move = true;
            }
            if (leftLinearMotor.getCurrentPosition() > 0 || rightLinearMotor.getCurrentPosition() > 0 /*&& downMagnetElevator.getState() == false*/) {
                downDegel = false;
                flag = false;
                ADondMove = true;
                MyElevator.move = true;
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
                } else if (downMagnetElevator.getState() == true && upDegel == false && downDegel == false){
                    leftLinearMotor.setTargetPosition(encodersStay);
                    rightLinearMotor.setTargetPosition(encodersStay);
                    leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLinearMotor.setPower(0.4);
                    rightLinearMotor.setPower(0.4);
                }else if (upDegel == false && downDegel == false){
                    leftLinearMotor.setPower(0);
                    rightLinearMotor.setPower(0);
                }

//            TODO: Levels:
/*            if (gamepad2.x){
                level1 = true;
                levels = false;
            }
            if (level1 == true){
                MyElevator.ElevateWithEncoder(20, 0.15, 0.01);
                stayingPosition = leftLinearMotor.getCurrentPosition();
            }
            if (leftLinearMotor.getCurrentPosition() < 24) {
                level1 = false;
                levels = true;
            }
*/
//            TODO: stop commands:
    /*        if (downMagnetElevator.getState() == false){
                underMagnet = true;
                leftLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }*/

          //  if (YDondMove == true && ADondMove == true && bumpersDondMove == true && underMagnet == false /*&& levels == true/*&& MyElevator.move == true*/) {
            /*    leftLinearMotor.setTargetPosition(encodersStay);
                rightLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(0.4);
                rightLinearMotor.setPower(0.4);
            }
            else if (underMagnet == true){
                leftLinearMotor.setPower(0);
                rightLinearMotor.setPower(0);
                telemetry.addLine("its working");
            }
*/
            encodersStay = stayingPosition;

//            TODO: telemetryes
            telemetry.addData("current Position LeftElevator", leftLinearMotor.getCurrentPosition());
            telemetry.addData("Left power Elevator", leftLinearMotor.getPower());
            telemetry.addData("right power Elevator", rightLinearMotor.getPower());
            telemetry.addData("Value under Magnet", underMagnet);
            telemetry.addData("down sensor Magnet", downMagnetElevator.getState());
            telemetry.update();

        }
    }
}