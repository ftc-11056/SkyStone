package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name = "tworiversTeleOp", group = "teamcode")
public class tworiversTeleOp extends Robot {

    private int stayingPosition = 0;
    private int encodersStay;
    private double time = 0;

    private boolean upDegel = false;
    private int downDegel = 0;
    private int downDegelToServo = 0;
    private boolean flag = false;
    private boolean bumpersDondMove = true;
    private boolean YDondMove = true;
    private boolean ADondMove = true;
    private boolean underMagnet = false;
    private boolean Abutton = false;
    //    private boolean levels = true;
//    private boolean level1 = false;
    private int anotherDownVar = 0;



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

        Arm.setPosition(0.135);

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
                MyDriveTrain.fieldOriented(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, heading);
            } else {
                MyDriveTrain.arcade(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
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
                Output.setPosition(0.1);
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
                Output.setPosition(0.1);
                telemetry.addData("time is:", time);
                //sleep(2000);
                telemetry.update();
            } else if (upDegel == true && downDegel != 1)
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
                Abutton = true;
                ADondMove = false;
                flag = true;
                downDegel = 1;
            }
            if (Abutton == true) {
                if (downDegel == 1 && anotherDownVar == 0) {
                    MyElevator.ElevateWithEncoder(-400, 0.3, 0.5);
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                }
                stayingPosition = leftLinearMotor.getCurrentPosition();

                if (rightLinearMotor.getCurrentPosition() < -220 || leftLinearMotor.getCurrentPosition() < -220) {
                    Arm.setPosition(0.135);

                }
                if (rightLinearMotor.getCurrentPosition() < -380 || leftLinearMotor.getCurrentPosition() < -380) {
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
                MyElevator.move = true;
            }
            if (gamepad2.left_bumper || gamepad2.right_bumper || leftLinearMotor.getCurrentPosition() > -0 || rightLinearMotor.getCurrentPosition() > -0
                    || downMagnetElevator.getState() == false) {
                downDegel = 0;
                downDegelToServo = 0;
                flag = false;
                ADondMove = true;
                MyElevator.move = true;
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
            } else if (downMagnetElevator.getState() == true && upDegel == false && downDegel == 0 && downDegelToServo == 0 && ADondMove && !gamepad2.a) {
                leftLinearMotor.setTargetPosition(encodersStay);
                rightLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(0.4);
                rightLinearMotor.setPower(0.4);
            } else if (upDegel == false && downDegel == 0 && downDegelToServo == 0 && ADondMove && !gamepad2.a) {
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
            telemetry.addData("downDegel", downDegel);
            telemetry.update();

        }
    }
}
