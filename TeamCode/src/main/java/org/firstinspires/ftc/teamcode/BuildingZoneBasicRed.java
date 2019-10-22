package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BuildingZoneBasicRed", group = "teamcode")
public class BuildingZoneBasicRed extends Robot {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
      //  LeftServo.setPosition(0.4);
     //   RightServo.setPosition(0.5);
        while (!isStarted())
        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();


        waitForStart();

        //angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Angles:", MyDriveTrain.getAngle());
        telemetry.update();
        telemetry.addData("Mikum:", Mikum);
        sleep(2000);
        telemetry.update();

        if (Mikum>2) {
            telemetry.addLine("you're on the right");
            telemetry.update();
        }
        else if (Mikum<-2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
        }
        else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
        }

        MyDriveTrain.encoderDrive(0.3, 25, 25, 25, 25);
        sleep(500);
        LeftServo.setPosition(0.75);
        RightServo.setPosition(0.25);
        sleep(1000);
        MyDriveTrain.encoderDrive(0.3, -25, -25, -25, -25);
        MyDriveTrain.encoderDrive(0.3, -40, 40, 40, -40);

    }

}
