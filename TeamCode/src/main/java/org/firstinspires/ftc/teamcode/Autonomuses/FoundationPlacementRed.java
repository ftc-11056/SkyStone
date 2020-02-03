package org.firstinspires.ftc.teamcode.Autonomuses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.basicAuto;


@Autonomous(name = "FoundationPlacementRed", group = "teamcode")
public class FoundationPlacementRed extends LinearOpMode {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override

    public void runOpMode() throws InterruptedException {
         DcMotor LB = null;
         DigitalChannel Touch_Foundation = null;
         DigitalChannel Touch_Foundation2 = null;
        LB = hardwareMap.get(DcMotor.class, "LB");
        Touch_Foundation = hardwareMap.get(DigitalChannel.class, "Touch_Foundation");
        Touch_Foundation2 = hardwareMap.get(DigitalChannel.class, "Touch_Foundation2");

        //  super.runOpMode();
        waitForStart();
        while (Touch_Foundation.getState() || (Touch_Foundation2.getState()) ){
      //      RB.setPower(-0.4);
            LB.setPower(-0.4);
      //      RF.setPower(-0.4);
      //      LF.setPower(-0.4);
        }


      /*  LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(500);
        MyDriveTrain.encoderDrive(1, 110, 110, 110, 110, 2);
        MyDriveTrain.encoderDrive(0.3, 140, 140, -140, -140, 2);
        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);
        ParkingMot.setPosition(ParkingMotIn);
        sleep(500);
        MyDriveTrain.encoderDrive(0.5, -10, -10, -10, -10, 1);
        MyDriveTrain.encoderDrive(0.5, 53, 53, 53, 53, 1);*/
    }

}


