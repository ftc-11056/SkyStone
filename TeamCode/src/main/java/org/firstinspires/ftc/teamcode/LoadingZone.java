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

        MyDriveTrain.encoderDrive(0.4,-20,20,20,-20);
        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Mikum>2) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.4,28,28,28,28);
            sleep(500);
            MyDriveTrain.encoderDrive(0.3,16,16,-16,-16);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.2,-30,-30,-30,-30);
            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.4,27,27,27,27);
            sleep(500);
            MyDriveTrain.encoderDrive(0.3,-16,-16,16,16);
            sleep(500);
            MyDriveTrain.encoderDrive(0.4,28,28,28,28);
            sleep(500);


        }
        else if (Mikum<-2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.4,13,13,13,13);
            sleep(500);
            MyDriveTrain.encoderDrive(0.4,-35,35,35,-35);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.4,-15,-15,-15,-15);
            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.4,32,-32,-32,32);
            sleep(500);
            MyDriveTrain.encoderDrive(0.4,40,40,40,40);
            sleep(500);

        }
        else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.4,18,18,18,18);
            sleep(500);
            MyDriveTrain.encoderDrive(0.4,-35,35,35,-35);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.4,-15,-15,-15,-15);
            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.4,30,-30,-30,30);
            sleep(500);
            MyDriveTrain.encoderDrive(0.4,40,40,40,40);
            sleep(500);
        }

        MyDriveTrain.encoderDrive(0.3,25,25,-25,-25);
        sleep(500);
        MyIntake.maxOuttake();
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.encoderDrive(0.2,-24,-24,24,24);
        sleep(500);

        if (Mikum>2) {
//            telemetry.addLine("you're on the right");
//            telemetry.update();
            MyDriveTrain.encoderDrive(0.4,-53,-53,-53,-53);
            sleep(500);
            MyDriveTrain.encoderDrive(0.4,-30,30,30,-30);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.4,-15,-15,-15,-15);
            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.4,30,-30,-30,30);
            sleep(500);
            MyDriveTrain.encoderDrive(0.4,40,40,40,40);
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

        MyDriveTrain.encoderDrive(0.3,25,25,-25,-25);
        sleep(500);
        MyIntake.maxOuttake();
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.encoderDrive(0.4,10,-10,-10,10);




    }

}
