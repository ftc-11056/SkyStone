package org.firstinspires.ftc.teamcode.Autonomuses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.basicAuto;


@Autonomous(name = "FoundationPlacementRed", group = "teamcode")
public class FoundationPlacementRed extends Robot {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        /*MyDriveTrain.encoderDrive(0.5,-30,30,30,-30,2);
        MyDriveTrain.encoderDrive(0.5,20,20,20,20,2);
        MyDriveTrain.RotateP(90,0.4,10,0.0108);
        MyDriveTrain.encoderDrive(0.5,40,40,40,40,2);
        MyDriveTrain.Rotate(90,0.4,10);
        LeftServo.setPosition(0.15);
        RightServo.setPosition(0.2);
        MyDriveTrain.encoderDrive(0.1,10,10,10,10,2);
        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(500);
        MyDriveTrain.encoderDrive(1,-90,-90,-90,-90,2);
        MyDriveTrain.Rotate(-0,0.5,10);
        MyDriveTrain.encoderDrive(1,40,40,40,40,2);
        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);
        MyDriveTrain.Rotate(0,0.4,10);
        MyDriveTrain.encoderDrive(0.3,-30,30,30,-30,2);
        ParkingMot.setPosition(ParkingMotIn);
        sleep(500);
        MyDriveTrain.encoderDrive(0.5,-43,-43, -43,-43,1);*/
        while (Touch_Foundation.getState()) {
            rightLinearMotor.setPower(0.5);
            leftLinearMotor.setPower(0.5);
//            if (Touch_Foundation.getState() == false){
//                rightLinearMotor.setPower(0);
//                leftLinearMotor.setPower(0);




        }

    }

}
