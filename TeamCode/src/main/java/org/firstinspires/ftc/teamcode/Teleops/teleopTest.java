package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name = "teleopCounterTest", group = "teleops")
@Disabled
public class teleopTest extends Robot {

    // levels counter
    private int currentLevel = 1;
    private int upEncodersToLevels = 0;
    private boolean flag = false;
    private int stayingPosition = 0;
    private int encodersStay = 0;
    private double stayPN = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        runtime.reset();

        rightLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

//        TODO: Levels Counter

            if (flag == false && gamepad2.dpad_up && (leftLinearMotor.getCurrentPosition() > -10 &&
                    leftLinearMotor.getCurrentPosition() < 10) /*&& upEncodersToLevels == 0*/) {
                MyElevator.ElevateWithEncoderTest(-100, 0.3, 0.9);
                stayingPosition = leftLinearMotor.getCurrentPosition();
                flag = true;
                if (leftLinearMotor.getCurrentPosition() <= -90) flag = false;
            }
            else if (!flag && gamepad2.dpad_up && (leftLinearMotor.getCurrentPosition() > -110 &&
                    leftLinearMotor.getCurrentPosition() < -90) /*&& upEncodersToLevels == -100*/) {
                MyElevator.ElevateWithEncoderTest(-200, 0.3, 0.9);
                stayingPosition = leftLinearMotor.getCurrentPosition();
                flag = true;
                if (leftLinearMotor.getCurrentPosition() <= -190) flag = false;
            }
            else if (!flag && gamepad2.dpad_up && (leftLinearMotor.getCurrentPosition() > -210 &&
                    leftLinearMotor.getCurrentPosition() < -190)/* && upEncodersToLevels == -200*/) {
                MyElevator.ElevateWithEncoderTest(-300, 0.3, 0.9);
                stayingPosition = leftLinearMotor.getCurrentPosition();
                flag = true;
                if (leftLinearMotor.getCurrentPosition() <= -290) flag = false;
            }
            else flag = false;

            encodersStay = stayingPosition;

            //     if (leftLinearMotor.getCurrentPosition() > -110 && leftLinearMotor.getCurrentPosition() < -90) upEncodersToLevels = -100;
            //    else if (leftLinearMotor.getCurrentPosition() > -210 && leftLinearMotor.getCurrentPosition() < -190) upEncodersToLevels = -200;

            telemetry.addData("Encoders", leftLinearMotor.getCurrentPosition());
            telemetry.addData("upEncodersToLevels", upEncodersToLevels);
            telemetry.addData("left Power", leftLinearMotor.getPower());
            telemetry.addData("right Power", rightLinearMotor.getPower());
            telemetry.update();
            if (MyDriveTrain.touchFoundtion  .getState() == false){
                telemetry.addLine("1 1 0 5 6   T H E  Y  B O T ");
                telemetry.update();
            }



        }
    }
}
