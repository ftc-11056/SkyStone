package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;


public class MenahemTeleop extends Robot {

    // levels counter
    private int Level = 100;
    int counter = 1;
    int pos = 0;
    int rightEncoderPosition = 0;
    int downPOS = 0;
    int upPOS = 0;
    private boolean Counterflag = false;
    private boolean low = false;
    private boolean up = false;

    private boolean inReset = false;

    private double ledTime = 0;
    private boolean ledColor = true;

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

    private int fixAuto = 0;

    // normal down mode
    private String ArmMode = "in";

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
            /*if (!Counterflag && gamepad2.dpad_left) {
                Counterflag = true;
                pos = counter * Level;
                MyElevator.ElevateWithEncoderTest(pos, 0.199, 0.003);
                rightEncoderPosition = leftLinearMotor.getCurrentPosition();
                stayingPosition = leftLinearMotor.getCurrentPosition();
            }
            if(Counterflag && leftLinearMotor.getCurrentPosition() >= pos - 10){
                Counterflag = false;
                counter += 1;
            }*/

//            TODO: One Level Lower
            if (gamepad2.dpad_down && !low){
                low = true;
                downPOS =  (counter - 1) * Level;
                MyElevator.ElevateWithEncoderTest(downPOS, 0.1, 0.003);
                stayingPosition = leftLinearMotor.getCurrentPosition();
            }
            if(low && leftLinearMotor.getCurrentPosition() <= downPOS + 10){
                low = false;
                counter -=1;
                stayingPosition = leftLinearMotor.getCurrentPosition();
            }

//            TODO: One Level Uper
            if (gamepad2.dpad_up && !up){
                up = true;
                upPOS =  (counter) * Level;
                MyElevator.ElevateWithEncoderTest(upPOS, 0.1, 0.003);
                stayingPosition = leftLinearMotor.getCurrentPosition();
            }
            if(up && leftLinearMotor.getCurrentPosition() >= upPOS - 10){
                up = false;
                counter += 1;
                stayingPosition = leftLinearMotor.getCurrentPosition();
            }

//            TODO: Reset Counter
            if (gamepad2.dpad_right) counter = 1;

//          TODO: YY Auto Button:
            if (!Counterflag && gamepad2.y) {
                Counterflag = true;
                pos = counter * Level;
            }
            if (gamepad2.y) {
                Output.setPosition(OutputDown);
                MyElevator.ElevateWithEncoder(400,1,1);
                YDondMove = false;
                upDegel = true;
                flag = true;
                time = runtime.seconds();
                inReset = false;
            }
            if (leftLinearMotor.getCurrentPosition() > 350 && upDegel == true) {
                Arm.setPosition(ArmOpen);
                telemetry.addLine("BENETTT");

            }
            if (upDegel == true && downDegel != 1){
                telemetry.addLine("gantzzzzzzzzzzzzzzzzzzzzzz");
            if ((runtime.seconds() -time) > 1.7) {
                    telemetry.addLine("bbbbbbbbbbbbbbbbbbbbbbbbb");
                    MyElevator.ElevateWithEncoder(1000, 0.1, 0.0088);
                    rightEncoderPosition = leftLinearMotor.getCurrentPosition();
              //      stayingPosition = leftLinearMotor.getCurrentPosition();
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
            }
            if (downDegel == 1 && upDegel != true)
                if ((-time + runtime.seconds()) > 1.7) {
                    MyElevator.ElevateWithEncoder(20, 0.1, 0.0088);
                    stayingPosition = leftLinearMotor.getCurrentPosition();
                    telemetry.addLine("Here");
                    telemetry.update();
                }

//            TODO: reset auto Buttons:
            if (gamepad2.right_bumper || gamepad2.left_bumper ||
                    gamepad2.dpad_left || gamepad2.dpad_right || (!inReset &&
                    (leftLinearMotor.getCurrentPosition() <= pos + 10) && Counterflag == true && Arm.getPosition() == ArmOpen)) {
                inReset=true;
                upDegel = false;
                flag = false;
                YDondMove = true;
                Counterflag = false;
                counter += 1;
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
            /*else if (downMagnetElevator.getState() == true && upDegel == false && downDegel == 0
                    && downDegelToServo == 0 && ADondMove && !gamepad2.a && up == false
                    && low == false) {
                telemetry.addLine("staying");
                stayErrors = leftLinearMotor.getCurrentPosition() - stayingPosition;
                power = 1 * stayErrors * stayPN;
                leftLinearMotor.setTargetPosition(encodersStay);
                leftLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinearMotor.setPower(power);
                rightLinearMotor.setPower(power);
            } else if (upDegel == false && downDegel == 0 && downDegelToServo == 0 && ADondMove && !gamepad2.a
                    && up == false && low == false){
                leftLinearMotor.setPower(0);
                rightLinearMotor.setPower(0);
            }*/

            encodersStay = stayingPosition;

            telemetry.addData("current Position LeftElevator", leftLinearMotor.getCurrentPosition());
            telemetry.addData("counter", counter);
//            telemetry.addData("FLAG", Counterflag);
            telemetry.addData("Time", time);
            telemetry.addData("Diffrence time", -time + runtime.seconds());
//            telemetry.addData("POS", pos);
//            telemetry.addData("LOW", low);
//            telemetry.addData("UP", up);
            telemetry.update();
        }
    }
}
