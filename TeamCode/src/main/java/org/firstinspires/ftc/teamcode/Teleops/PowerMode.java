package org.firstinspires.ftc.teamcode.Teleops;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotCustomade;

@TeleOp(name = "powerMode" , group = "teleops")
//@Disabled
public class PowerMode extends RobotCustomade {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Output.setPosition(OutputClose);

        leftLinearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {

            /*if (gamepad2.b) rightLinearMotor.setPower(0.5);
            else if (gamepad2.x) rightLinearMotor.setPower(-0.5);
            else if (gamepad2.y) leftLinearMotor.setPower(0.5);
            else if (gamepad2.a) leftLinearMotor.setPower(-0.5);
            else*/ if (gamepad2.dpad_up) {
                leftLinearMotor.setPower(1);
                rightLinearMotor.setPower(1);
            } else if (gamepad2.dpad_down) {
                leftLinearMotor.setPower(-1);
                rightLinearMotor.setPower(-1);
            } else {
                leftLinearMotor.setPower(0);
                rightLinearMotor.setPower(0);
            }

            if (gamepad2.y) Output.setPosition(OutputOpen);
            else if (gamepad2.a) Output.setPosition(OutputClose);

           /* if (gamepad2.right_bumper) rightLinearMotor.setPower(0.3);
            else if (gamepad2.left_bumper) leftLinearMotor.setPower(0.3);*/

            if (gamepad1.x) LB.setPower(0.5);
            else if (gamepad1.y) LF.setPower(0.5);
            else if (gamepad1.a) RB.setPower(0.5);
            else if (gamepad1.b) RF.setPower(0.5);
            else {
                LB.setPower(0);
                LF.setPower(0);
                RB.setPower(0);
                RF.setPower(0);
            }

            if (gamepad1.dpad_up) RightServo.setPosition(0.8);
            if (gamepad1.dpad_down) RightServo.setPosition(0.2);
            if (gamepad1.dpad_left) LeftServo.setPosition(0.8);
            if (gamepad1.dpad_right) LeftServo.setPosition(0.2);
        }
    }
}