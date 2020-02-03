package org.firstinspires.ftc.teamcode.Autonomuses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.basicAuto;

@Autonomous(name = "FoundationPlacementBlue", group = "teamcode")
public class FoundationPlacementBlue extends basicAuto {

    /* Declare OpMode members. */


    /**
     * Code to run ONCE when the driver hits INIT
     **/
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
      /* while (Touch_Foundation.getState()) {
            RB.setPower(-0.4);
            LB.setPower(-0.4);
            RF.setPower(-0.4);
            LF.setPower(-0.4);
        }*/
        MyDriveTrain.encoderDrive(1, -50, -50, -50, -50, 2);
        LeftServo.setPosition(LeftServoDown);
            RightServo.setPosition(RightServoDown);
            MyDriveTrain.encoderDrive(1, 110, 110, 110, 110, 2);
            MyDriveTrain.encoderDrive(0.3, -140, -140, 140, 140, 2);
            LeftServo.setPosition(LeftServoUp);
            RightServo.setPosition(RightServoUp);
            ParkingMot.setPosition(ParkingMotIn);
            sleep(500);
            MyDriveTrain.encoderDrive(0.5, -10, -10, -10, -10, 1);
            MyDriveTrain.encoderDrive(0.5, 53, 53, 53, 53, 1);


        }

    }

