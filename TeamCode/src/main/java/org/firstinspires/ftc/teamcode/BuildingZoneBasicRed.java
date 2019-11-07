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
        waitForStart();
        MyDriveTrain.encoderDrive(0.5,75,75,75,75,2);
        sleep(1000);
        LeftServo.setPosition(0);
        RightServo.setPosition(0.25);
        sleep(1000);
        MyDriveTrain.encoderDrive(0.5,-128,-128,-134,-134,2);
        MyDriveTrain.encoderDrive(0.5,110,110,-110,-110,2);
        sleep(2000);
        LeftServo.setPosition(0.6);
        RightServo.setPosition(0.9);
        MyDriveTrain.encoderDrive(0.5,-75,-75, -75,-75,1);
        telemetry.addData("Angles:", MyDriveTrain.getAngle());
        telemetry.update();


    }

}
