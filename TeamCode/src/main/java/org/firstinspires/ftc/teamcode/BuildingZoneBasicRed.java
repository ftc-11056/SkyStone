package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BuildingZoneBasicRed", group = "teamcode")
public class BuildingZoneBasicRed extends basicAuto {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        LeftServo.setPosition(0);
        RightServo.setPosition(0.25);

        waitForStart();
        MyDriveTrain.encoderDrive(0.5,-100,-100,-100,-100,2);
        LeftServo.setPosition(0.6);
        RightServo.setPosition(0.9);
        MyDriveTrain.encoderDrive(0.5,90,90,90,90,2);
        LeftServo.setPosition(0);
        RightServo.setPosition(0.25);
        MyDriveTrain.encoderDrive(0.5,-70,70,70,-70,1);
        telemetry.addData("Angles:", MyDriveTrain.getAngle());
        telemetry.update();


    }

}
