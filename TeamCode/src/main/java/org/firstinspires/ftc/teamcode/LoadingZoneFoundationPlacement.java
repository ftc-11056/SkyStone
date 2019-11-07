package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "LoadingZoneFoundationPlacement", group = "teamcode")
public class LoadingZoneFoundationPlacement extends basicAuto {

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

        MyDriveTrain.encoderDrive(0.8, -30, 30, 30, -30, 1);
        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Mikum > 2) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            MyDriveTrain.encoderDrive(1, 85, 85, 85, 85, 2);
            MyDriveTrain.encoderDrive(1, 34, 34, -34, -34, 2);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.7, -80, -80, -80, -80, 2);
            if (cubeIn.getDistance(DistanceUnit.MM) > 60) {
                sleep(500);
                MyDriveTrain.encoderDrive(0.7, -20, -20, -20, -20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(1, 60, 60, 60, 60, 2);
            MyDriveTrain.encoderDrive(1, -37, -37, 37, 37, 2);
            MyDriveTrain.encoderDrive(1, 70, 70, 70, 70, 2);


        } else if (Mikum < -2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.encoderDrive(1, 43, 43, 43, 43, 2);
            MyDriveTrain.encoderDrive(0.8, -90, 90, 90, -90, 1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1, -25, -25, -25, -25, 2);
            if (cubeIn.getDistance(DistanceUnit.MM) > 60) {
                sleep(500);
                MyDriveTrain.encoderDrive(0.7, -20, -20, -20, -20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 65, -65, -65, 65, 1);
            MyDriveTrain.encoderDrive(1, 120, 120, 120, 120, 2);


        } else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(1, 43, 43, 43, 43, 2);
            MyDriveTrain.encoderDrive(0.8, -90, 90, 90, -90, 1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1, -25, -25, -25, -25, 2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > 60) {
                sleep(500);
                MyDriveTrain.encoderDrive(0.7, -20, -20, -20, -20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 65, -65, -65, 65, 1);
            MyDriveTrain.encoderDrive(1, 120, 120, 120, 120, 2);
        }

//    The system catches the Skystone

        MyDriveTrain.encoderDrive(1, 80, 80, 80, 80, 2);
        MyDriveTrain.encoderDrive(0.8, -60, -60, 60, 60, 2);
        MyDriveTrain.encoderDrive(0.5, 30, 30, 30, 30, 2);
        sleep(1000);
        LeftServo.setPosition(0);
        RightServo.setPosition(0.25);
        sleep(1000);
//    The system moves the Skystone to the Foundation
        MyDriveTrain.encoderDrive(1, -100, -100, -100, -100, 2);
        LeftServo.setPosition(0.6);
        RightServo.setPosition(0.9);
        sleep(1000);
        MyDriveTrain.encoderDrive(0.8, -120, 120, 120, -120, 1);

    }

}
