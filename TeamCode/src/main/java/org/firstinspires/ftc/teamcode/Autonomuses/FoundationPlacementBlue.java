package org.firstinspires.ftc.teamcode.Autonomuses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.basicAuto;

@Disabled
@Autonomous(name = "FoundationPlacementBlue", group = "teamcode")
public class FoundationPlacementBlue extends basicAuto {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
       // programRunTime.reset();
        MyDriveTrain.encoderDrive(0.5,-30,30,30,-30,2);
        MyDriveTrain.encoderDrive(0.5,-20,-20,-20,-20,2);
        MyDriveTrain.RotateP(90,0.7,10,0.0108);
        MyDriveTrain.encoderDrive(0.5,40,40,40,40,2);
        MyDriveTrain.Rotate(90,0.1,10);
        LeftServo.setPosition(0.15);
        RightServo.setPosition(0.2);
        MyDriveTrain.encoderDrive(0.1,10,10,10,10,2);
        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(500);
        MyDriveTrain.encoderDrive(1,-90,-90,-90,-90,2);
        MyDriveTrain.Rotate(180,0.5,10);
        MyDriveTrain.encoderDrive(1,40,40,40,40,2);
        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);
        MyDriveTrain.Rotate(180,0.4,10);
        MyDriveTrain.encoderDrive(0.3,30,-30,-30,30,2);
        ParkingMot.setPosition(ParkingMotIn);
        sleep(500);
        MyDriveTrain.encoderDrive(0.5,-43,-43, -43,-43,1);


    }

}
