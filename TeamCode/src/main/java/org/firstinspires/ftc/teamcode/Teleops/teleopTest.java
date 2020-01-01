package org.firstinspires.ftc.teamcode.Teleops;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name = "teleopCounterTest", group = "teleops")
public class teleopTest extends Robot {

    // levels counter
    private int currentLevel = 1;
    private int upEncodersToLevels = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        runtime.reset();

        waitForStart();
        while (opModeIsActive()) {

//        TODO: Levels Counter

            if (gamepad2.dpad_up && (leftLinearMotor.getCurrentPosition() > -10 && leftLinearMotor.getCurrentPosition() < 10) && upEncodersToLevels == 0) {
                MyElevator.ElevateWithEncoder(-100,0.3,1);
            }else if (gamepad2.dpad_up && (leftLinearMotor.getCurrentPosition() > -110 && leftLinearMotor.getCurrentPosition() < -90) && upEncodersToLevels == 100){
                MyElevator.ElevateWithEncoder(-200,0.3,1);
            }else if (gamepad2.dpad_up && (leftLinearMotor.getCurrentPosition() > -210 && leftLinearMotor.getCurrentPosition() < -190) && upEncodersToLevels == 200){
                MyElevator.ElevateWithEncoder(-300,0.3,1);
            }else if (gamepad2.dpad_up && (leftLinearMotor.getCurrentPosition() > -310 && leftLinearMotor.getCurrentPosition() < -290) && upEncodersToLevels == 300){
                MyElevator.ElevateWithEncoder(-400,0.3,1);
            }

            if (leftLinearMotor.getCurrentPosition() > -110 && leftLinearMotor.getCurrentPosition() < -90) upEncodersToLevels = -100;
            else if (leftLinearMotor.getCurrentPosition() > -210 && leftLinearMotor.getCurrentPosition() < -190) upEncodersToLevels = -200;
            else if (leftLinearMotor.getCurrentPosition() > -310 && leftLinearMotor.getCurrentPosition() < -290) upEncodersToLevels = -300;

        telemetry.addData("Encoders", leftLinearMotor.getCurrentPosition());
        telemetry.addData("upEncodersToLevels", upEncodersToLevels);
        telemetry.update();

        }
    }
}
