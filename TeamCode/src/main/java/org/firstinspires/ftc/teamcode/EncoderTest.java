package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "EncoderTest", group = "teamcode")
@Disabled

public class EncoderTest extends Robot {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //  LeftServo.setPosition(0.4);
        //   RightServo.setPosition(0.5);

        waitForStart();
        runtime.reset();

                MyDriveTrain.encoderDrive(1,-120,120,120,-120,1);
        while (!isStopRequested()) {
            telemetry.addData("LFencoder",LF.getCurrentPosition());
            telemetry.update();
        }

    }
    }

