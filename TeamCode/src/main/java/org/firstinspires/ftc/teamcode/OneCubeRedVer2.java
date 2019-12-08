package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "OneCubeRedVer2", group = "teamcode")
@Disabled
public class OneCubeRedVer2 extends basicAuto {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //  LeftServo.setPosition(0.4);
        //   RightServo.setPosition(0.5);

        passWord = "pass";


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
        MyDriveTrain.Rotate(0,0.1,10);
        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();


        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (Mikum > 2) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, -84, 84, 84, -84, 1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.8, -25, -25, -25, -25, 2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > 70) {
                MyDriveTrain.encoderDrive(0.7, -20, -20, -20, -20, 2);
                MyDriveTrain.encoderDrive(0.7, 20, 20, 20, 20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 90, -90, -90, 90, 1);
            MyDriveTrain.encoderDrive(0.8, 100, 100, 100, 100, 2);
        }

         else if (Mikum < -2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, 28, 28, 28, 28, 2);
            MyDriveTrain.encoderDrive(0.8, -88, 88, 88, -88, 1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1, -25, -25, -25, -25, 2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > 60) {
                sleep(500);
                MyDriveTrain.encoderDrive(0.7, -20, -20, -20, -20, 2);
                MyDriveTrain.encoderDrive(0.7, 20, 20, 20, 20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 90, -90, -90, 90, 1);
            MyDriveTrain.encoderDrive(1, 85, 85, 85, 85, 2);



        } else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(1, 41, 41, 41, 41, 2);
            MyDriveTrain.encoderDrive(0.8, -88, 88, 88, -88, 1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1, -25, -25, -25, -25, 2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > 60) {
                MyDriveTrain.encoderDrive(0.7, -20, -20, -20, -20, 2);
                MyDriveTrain.encoderDrive(0.7, 20, 20, 20, 20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 90, -90, -90, 90, 1);
            MyDriveTrain.Rotate(0,0.1,10);
            MyDriveTrain.encoderDrive(1, 80, 80, 80, 80, 2);

        }

        MyDriveTrain.Rotate(0,0.2,10);
        MyIntake.maxIntake();
        sleep(500);
        MyIntake.ShutDown();
        Output.setPosition(OutputDown);
        MyDriveTrain.encoderDrive(1, 125, 125, 125, 125, 2);
        MyDriveTrain.encoderDrive(0.3,30,-30,-30,30,2);
        MyDriveTrain.encoderDrive(0.5,-25,25,25,-25,2);
        MyDriveTrain.Rotate(90,0.1,10);
        sleep(500);
        MyDriveTrain.encoderDrive(0.2,60,60,60,60,2);
        MyDriveTrain.Rotate(90,0.1,10);
        LeftServo.setPosition(0.15);
        RightServo.setPosition(0.15);
        MyDriveTrain.encoderDrive(0.1,10,10,10,10,2);
        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(500);
        MyDriveTrain.encoderDrive(1,-90,-90,-90,-90,2);
        MyDriveTrain.Rotate(-0,1,10);
        MyDriveTrain.encoderDrive(1,90,90,90,90,2);
        MyElevator.ElevateWithEncoder(-350, 0.3, 0.5);
        Arm.setPosition(1);
        sleep(1000);
        MyElevator.ElevateWithEncoder(0, 0.1, 0.001);
        Output.setPosition(OutputUp);
        sleep(1000);
        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);
        MyDriveTrain.Rotate(0,0.4,10);
        MyDriveTrain.encoderDrive(0.3,25,-25,-25,25,2);
        MyDriveTrain.encoderDrive(0.5,-40,-40, -40,-40,1);
        ParkingMot.setPosition(ParkingMotOut);
    }

}
