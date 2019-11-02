package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "EncoderTest", group = "teamcode")

public class EncoderTest extends Robot {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //  LeftServo.setPosition(0.4);
        //   RightServo.setPosition(0.5);

        waitForStart();
        MyDriveTrain.encoderDrive(1,-120,-120,-120,-120);
        sleep(10000);
    }
    }

