package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name = "MenahemTeleop", group = "teleops")
public class MenahemTeleop extends Robot {

    // levels counter
    private int Level = 100;
    int counter = 1;
    int pos = 0;
    int rightEncoderPosition = 0;
    int downPOS = 0;
    int upPOS = 0;
    private boolean flag = false;
    private boolean low = false;
    private boolean up = false;

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
    private boolean bumpersDondMove = true;
    private boolean ADondMove = true;
    private boolean underMagnet = false;
    private boolean Abutton = false;
    private int anotherDownVar = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        runtime.reset();

        rightLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        while (opModeIsActive()) {


//        TODO: Levels Counter

//            TODO: change the next Level
            if (!flag && gamepad1.dpad_left) {
                flag = true;
                pos = counter * Level;
                MyElevator.ElevateWithEncoderTest(pos, 0.199, 0.003);
                rightEncoderPosition = leftLinearMotor.getCurrentPosition();
            }
            if(flag && leftLinearMotor.getCurrentPosition() >= pos - 10){
                flag = false;
                counter += 1;
            }

//            TODO: One Level Lower
            if (gamepad1.dpad_down && !low){
                low = true;
                downPOS =  (counter - 1) * Level;
                MyElevator.ElevateWithEncoderTest(downPOS, 0.1, 0.003);
            }
            if(low && leftLinearMotor.getCurrentPosition() <= downPOS + 10){
                low = false;
                counter -=1;
            }

//            TODO: One Level Uper
            if (gamepad1.dpad_up && !up){
                up = true;
                upPOS =  (counter) * Level;
                MyElevator.ElevateWithEncoderTest(upPOS, 0.1, 0.003);
            }
            if(up && leftLinearMotor.getCurrentPosition() >= upPOS - 10){
                up = false;
                counter += 1;
            }

//            TODO: Reset Counter
            if (gamepad1.dpad_right) counter = 1;

//            TODO: A
            //          TODO: YY Auto Button:
            if (gamepad2.y) {
                YDondMove = false;
                upDegel = true;
                flag = true;
                underMagnet = false;
                if (upDegel == true) {
                    time = runtime.seconds();
                }
                Output.setPosition(OutputDown);
                telemetry.addData("time is:", time);
                telemetry.update();
            } else if (upDegel == true && downDegel != 1)
                if ((-time + runtime.seconds()) > 0.7) {
                    MyElevator.ElevateWithEncoder(-400, 1, 0.8);
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                    if (leftLinearMotor.getCurrentPosition() < -350) {
                        Arm.setPosition(1);
                    }
                }
//          TODO: AA Auto Button:
            if (gamepad2.a) {
                Output.setPosition(OutputUp);
                MyElevator.ElevateWithEncoder(-570,1,1);
                ADondMove = false;
                downDegel = 1;
                flag = true;
                time = runtime.seconds();
            }
            if (leftLinearMotor.getCurrentPosition() < -350 && downDegel == 1) {
                Arm.setPosition(ArmClose);
                telemetry.addData("time is:", time);
                telemetry.update();
            }
            if (downDegel == 1 && upDegel != true)
                if ((-time + runtime.seconds()) > 1.7) {
                    MyElevator.ElevateWithEncoder(20, 0.1, 0.0088);
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                    telemetry.addLine("Here");
                    telemetry.update();
                }

//            TODO: reset auto Buttons:
            if (leftLinearMotor.getCurrentPosition() < -380 || gamepad2.right_bumper || gamepad2.left_bumper ||
                    gamepad2.dpad_left || gamepad2.dpad_right) {
                upDegel = false;
                flag = false;
                YDondMove = true;
            }
            if (gamepad2.left_bumper || gamepad2.right_bumper || leftLinearMotor.getCurrentPosition() > -0 ||
                    downMagnetElevator.getState() == false || gamepad2.right_bumper || gamepad2.left_bumper) {
                downDegel = 0;
                downDegelToServo = 0;
                flag = false;
                ADondMove = true;
                Abutton = false;
            }

            if (gamepad2.left_bumper && downMagnetElevator.getState() == true) {
                MyElevator.ElevateWithEncoder(20, 0.1, 0.0088);
                stayingPosition = leftLinearMotor.getCurrentPosition();
                bumpersDondMove = false;
            }
//            TODO: stop commands
            else if (downMagnetElevator.getState() == true && upDegel == false && downDegel == 0 && downDegelToServo == 0 && ADondMove && !gamepad2.a) {
                stayErrors = leftLinearMotor.getCurrentPosition() - stayingPosition;
//                stayCounter = stayCounter + leftLinearMotor.getCurrentPosition(+ );
                power = 1 * stayErrors * stayPN /*+ stayCounter * stayDN*/;
                leftLinearMotor.setTargetPosition(encodersStay);
                //    rightLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // rightLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(power);
                rightLinearMotor.setPower(power);
            } else if (upDegel == false && downDegel == 0 && downDegelToServo == 0 && ADondMove && !gamepad2.a) {
                leftLinearMotor.setPower(0);
                rightLinearMotor.setPower(0);
            }

            encodersStay = stayingPosition;

            telemetry.addData("current Position LeftElevator", leftLinearMotor.getCurrentPosition());
            telemetry.addData("FLAG", flag);
            telemetry.addData("LOW", low);
            telemetry.addData("UP", up);
            telemetry.addData("POS", pos);
            telemetry.addData("counter", counter);
            telemetry.update();
        }
    }
}
