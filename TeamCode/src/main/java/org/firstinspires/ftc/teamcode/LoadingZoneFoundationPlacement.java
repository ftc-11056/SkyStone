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
            MyDriveTrain.encoderDrive(1, -40, -40, 40, 40, 2);
            MyDriveTrain.encoderDrive(1, 80, 80, 80, 80, 2);


        } else if (Mikum < -2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.encoderDrive(1, 30, 30, 30, 30, 2);
            MyDriveTrain.encoderDrive(0.8, -88, 88, 88, -88, 1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1, -25, -25, -25, -25, 2);
            if (cubeIn.getDistance(DistanceUnit.MM) > 60) {
                sleep(500);
                MyDriveTrain.encoderDrive(0.7, -20, -20, -20, -20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 70, -70, -70, 70, 1);
            MyDriveTrain.encoderDrive(1, 140, 140, 140, 140, 2);


        } else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(1, 43, 43, 43, 43, 2);
            MyDriveTrain.encoderDrive(0.8, -88, 88, 88, -88, 1);
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
            MyDriveTrain.encoderDrive(0.8, 70, -70, -70, 70, 1);
            MyDriveTrain.encoderDrive(1, 120, 120, 120, 120, 2);
        }

//        Output.setPosition(0.25);
        MyDriveTrain.encoderDrive(1, 65, 65, 65, 65, 2);
        MyDriveTrain.encoderDrive(0.5,65,-65,-65,65,2);
        MyDriveTrain.encoderDrive(0.5,-25,25,25,-25,2);
        MyDriveTrain.Rotate(90,0.1,10);
        sleep(500);
        MyDriveTrain.encoderDrive(0.2,65,65,65,65,2);
        sleep(1000);
//        MyElevator.ElevateWithEncoder(-350, 0.3, 0.5);
//        Arm.setPosition(1);
//        sleep(1000);
//        MyElevator.ElevateWithEncoder(8, 0.4, 0.007);
//        sleep(1000);
//        Output.setPosition(1);
        sleep(10000);
//        LeftServo.setPosition(0);
//        RightServo.setPosition(0.25);
        sleep(1000);
        MyDriveTrain.encoderDrive(1,-128,-128,-138,-138,2);
        MyDriveTrain.encoderDrive(1,110,110,-110,-110,2);
        MyElevator.ElevateWithEncoder(-350, 0.3, 0.5);
        Arm.setPosition(1);
        sleep(1000);
        MyElevator.ElevateWithEncoder(10, 0.4, 0.007);
        Output.setPosition(1);
        sleep(1000);
        LeftServo.setPosition(0.6);
        RightServo.setPosition(0.9);
        MyDriveTrain.Rotate(0,0.4,10);
        sleep(1000);
        MyDriveTrain.encoderDrive(0.5,-75,-75, -75,-75,1);

    }

}
