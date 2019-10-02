package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="TeleOpTest", group="teamcode")
public class TeleOpTest extends Robot {


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        while (opModeIsActive()) {

            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = angles.firstAngle;


            if (gamepad1.x) MyDriveTrain.setMode("arcade");
            else if (gamepad1.a) MyDriveTrain.setMode("Oriented");

            telemetry.addData("Mode: ", MyDriveTrain.Mode);
            telemetry.update();

            if (MyDriveTrain.getMode().equals("Oriented")) {
                MyDriveTrain.fieldOriented(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, heading);
            } else {
                MyDriveTrain.arcade(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            if (gamepad1.left_bumper /*&& Arm.getPosition() > 0.75*/) {
                Arm.setPosition(Arm.getPosition() - servoPosition);
            }
              else if (gamepad1.right_bumper /*&& Arm.getPosition() < 1*/) {
                Arm.setPosition(Arm.getPosition() + servoPosition);
            }

            else{
                Arm.setPosition(Arm.getPosition());
            }
            telemetry.addData("Arm Position", Arm.getPosition());
            telemetry.update();

            if (gamepad1.a) {
                Intake.setPosition(1);

            }
            else if (gamepad1.y) {
                Intake.setPosition(0);
            }



        }

    }
}




