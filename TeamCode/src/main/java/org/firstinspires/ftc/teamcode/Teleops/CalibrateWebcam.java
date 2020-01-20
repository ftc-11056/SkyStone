package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.basicAuto;

@TeleOp(name = "CalibrateWebcam",group = "dogecv")
public class CalibrateWebcam extends basicAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        while (opModeIsActive()){
            /*if (gamepad1.x) skystoneDetector.downBlack = (int) Math.ceil(-gamepad1.left_stick_y * 100);
            if (gamepad1.a) skystoneDetector.upBlack = (int) Math.ceil(-gamepad1.right_stick_y * 100);
            if (gamepad1.b) skystoneDetector.yellow = (int) Math.ceil(-gamepad2.left_stick_y * 100);

            if (gamepad1.left_stick_y < 0 || gamepad1.right_stick_y < 0 || gamepad2.left_stick_y < 0){
                telemetry.addData("111LeftStick", -gamepad1.left_stick_y * 100);
                telemetry.addData("111RightStick", -gamepad1.right_stick_y * 100);
                telemetry.addData("222LeftStick", -gamepad2.left_stick_y * 100);
            }else {
                telemetry.addData("downBlack", skystoneDetector.downBlack);
                telemetry.addData("upBlack", skystoneDetector.upBlack);
                telemetry.addData("Yellow", skystoneDetector.yellow);
            }*/
            telemetry.addData("X", skystoneDetector.getScreenPosition().x);
            telemetry.addData("Y", skystoneDetector.getScreenPosition().y);
            telemetry.update();
        }
    }
}
