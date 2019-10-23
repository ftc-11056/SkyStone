package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "LoadingZone", group = "teamcode")
public class LoadingZone extends Robot {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //  LeftServo.setPosition(0.4);
        //   RightServo.setPosition(0.5);

        waitForStart();

        telemetry.addData("Angles:", MyDriveTrain.getAngle());
        telemetry.update();
        telemetry.addData("Mikum:", Mikum);
        telemetry.update();

        MyDriveTrain.encoderDrive(0.5,30,-30,-30,30);

        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();

        if (Mikum>2) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.5,30,-30,-30,30);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.5,20,20,20,20);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.5,-30,30,30,-30);
            MyDriveTrain.encoderDrive(0.5,25,25,25,25);
        }
        else if (Mikum<-2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.Rotate(50,0.4,5);
            MyDriveTrain.encoderDrive(0.5,30,-30,-30,30);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.5,20,20,20,20);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.5,-30,30,30,-30);
            MyDriveTrain.Rotate(-180,0.4,5);
            MyDriveTrain.encoderDrive(0.5,25,25,25,25);

        }
        else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.5,30,30,30,30);
            sleep(500);
            MyDriveTrain.encoderDrive(0.5,50,-50,-50,50);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.5,-20,-20,-20,-20);
            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.5,-35,35,35,-35);
            sleep(500);
            MyDriveTrain.encoderDrive(0.5,-70,-70,-70,-70);
            sleep(500);
        }

        MyIntake.maxIntake();
        sleep(1000);
        MyIntake.ShutDown();


    }

}
