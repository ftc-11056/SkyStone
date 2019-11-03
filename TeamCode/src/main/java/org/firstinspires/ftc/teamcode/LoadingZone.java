package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "LoadingZone", group = "teamcode")
public class LoadingZone extends basicAuto {

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

        telemetry.addData("Angles", MyDriveTrain.getAngle());
        telemetry.update();
        telemetry.addData("Mikum:", Mikum);
        telemetry.update();
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MyDriveTrain.encoderDrive(0.7,-30,30,30,-30 ,1);
        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Mikum>2) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.7,38,38,38,38,2);
            sleep(500);
            MyDriveTrain.encoderDrive(0.6,24,24,-24,-24,2);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.7,-40,-40,-40,-40,2);
            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.7,37,37,37,37,2);
            sleep(500);
            MyDriveTrain.encoderDrive(0.6,-26,-26,26,26,2);
            sleep(500);
            MyDriveTrain.encoderDrive(0.7,38,38,38,38,2);
            sleep(500);


        }
        else if (Mikum<-2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.7,23,23,23,23,2);
            sleep(500);
            MyDriveTrain.encoderDrive(0.7,-45,45,45,-45,1);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.7,-25,-25,-25,-25,2);
            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.7,42,-42,-42,42,1);
            sleep(500);
            MyDriveTrain.encoderDrive(0.7,50,50,50,50,2);
            sleep(500);

        }
        else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.7,28,28,28,28,2);
            sleep(500);
            MyDriveTrain.encoderDrive(0.7,-45,45,45,-45,1);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.7,-25,-25,-25,-25,2);
            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.7,40,-40,-40,40,1);
            sleep(500);
            MyDriveTrain.encoderDrive(0.7,50,50,50,50,2);
            sleep(500);
        }

        MyDriveTrain.encoderDrive(0.6,35,35,-35,-35,2);
        sleep(500);
        MyIntake.maxOuttake();
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.encoderDrive(0.6,-34,-34,34,34,2);
        sleep(500);

        if (Mikum>2) {
//            telemetry.addLine("you're on the right");
//            telemetry.update();
            MyDriveTrain.encoderDrive(0.7,-63,-63,-63,-63,2);
            sleep(500);
            MyDriveTrain.encoderDrive(0.7,-40,40,40,-40,1);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.7,-25,-25,-25,-25,2);
            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.7,40,-40,-40,40,1);
            sleep(500);
            MyDriveTrain.encoderDrive(0.7,50,50,50,50,2);
            sleep(500);
        }
        else if (Mikum<-2) {
//            telemetry.addLine("you're on the left");
//            telemetry.update();


        }
        else {
//            telemetry.addLine("You are on the center!");
//            telemetry.update();

        }

        MyDriveTrain.encoderDrive(0.6,35,35,-35,-35,2);
        sleep(500);
        MyIntake.maxOuttake();
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.encoderDrive(0.7,20,-20,-20,20,1);




    }

}
