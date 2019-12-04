package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "TwoCubeRed", group = "teamcode")
public class TwoCubeRed extends basicAuto {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        telemetry.addData("Angles", MyDriveTrain.getAngle());
        telemetry.update();
        telemetry.addData("Mikum:", Mikum);
        telemetry.update();
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MyDriveTrain.encoderDrive(0.8,-30,30,30,-30 ,1);
        MyDriveTrain.Rotate(0,0.1,10);
        Output.setPosition(OutputDown);
        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();
        Mikum = 3;

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Mikum>2) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8,-35,35,35,-35 ,1);
            MyDriveTrain.encoderDrive(1,70,70,70,70,2);
            MyDriveTrain.RotateP(-38,0.7,10,0.0108);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.2,-60,-60,-60,-60,2);
            sleep(500);
//            if (cubeIn.getDistance(DistanceUnit.MM) > 60){
//                MyDriveTrain.encoderDrive(0.7,-20,-20,-20,-20,2);
//                telemetry.addData("CubeDistans:",cubeIn.getDistance(DistanceUnit.MM));
//                telemetry.update();
//            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(1,50,50,50,50,2);
            MyDriveTrain.Rotate(0,0.1,10);
            MyDriveTrain.encoderDrive(1,80,80,80,80,2);


        }
        else if (Mikum<-2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, 25, 25, 25, 25, 2);
            MyDriveTrain.encoderDrive(0.8, -86, 86, 86, -86, 1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.8, -25, -25, -25, -25, 2);
            sleep(500);
//            if (cubeIn.getDistance(DistanceUnit.MM) > 70) {
//                sleep(500);
//                MyDriveTrain.encoderDrive(0.5, -20, -20, -20, -20, 2);
//                MyDriveTrain.encoderDrive(0.7, 20, 20, 20, 20, 2);
//                telemetry.addData("CubeDistance:", cubeIn.getDistance(DistanceUnit.MM));
//                telemetry.update();
//            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 45, -45, -45, 45, 1);
            MyDriveTrain.Rotate(0,0.1,10);
            MyDriveTrain.encoderDrive(1, 100, 100, 100, 100, 2);


        }
        else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(1, 30, 30, 30, 30, 2);
            MyDriveTrain.encoderDrive(0.8, -84, 84, 84, -84, 1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.8, -25, -25, -25, -25, 2);
            sleep(500);
//            if (cubeIn.getDistance(DistanceUnit.MM) > 70) {
//                MyDriveTrain.encoderDrive(0.7, -20, -20, -20, -20, 2);
//                MyDriveTrain.encoderDrive(0.5, 20, 20, 20, 20, 2);
//                telemetry.addData("CubeDistance:", cubeIn.getDistance(DistanceUnit.MM));
//                telemetry.update();
//            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 45, -45, -45, 45, 1);
            MyDriveTrain.Rotate(0,0.1,10);
            MyDriveTrain.encoderDrive(1, 120, 120, 120, 120, 2);

        }

        MyDriveTrain.RotateP(-90,1,10,0.0208);
        MyIntake.maxOuttake();
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.RotateP(0,1,10,0.0108);


        if (Mikum>2) {
//            telemetry.addLine("you're on the right");
//            telemetry.update();
            MyDriveTrain.encoderDrive(1,-73,-73,-73,-73,2);
            MyDriveTrain.Rotate(0,0.1,10);
            MyDriveTrain.encoderDrive(1,-78,-78,-78,-78,2);
            MyDriveTrain.Rotate(0,0.1,10);
            MyDriveTrain.encoderDrive(1,-65,65,65,-65,1);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1,-25,-25,-25,-25,2);
            sleep(500);
//            if (cubeIn.getDistance(DistanceUnit.MM)>60){
//                MyDriveTrain.encoderDrive(1,-20,-20,-20,-20,2);
//                telemetry.addData("CubeDistans:",cubeIn.getDistance(DistanceUnit.MM));
//                telemetry.update();
//            }

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8,45,-45,-45,45,1);
            MyDriveTrain.Rotate(0,0.1,10);
            MyDriveTrain.encoderDrive(1,90,90,90,90,2);
            MyDriveTrain.Rotate(0,0.1,10);
            MyDriveTrain.encoderDrive(1,75,75,75,75,2);

        }
        else if (Mikum<-2) {
//            telemetry.addLine("you're on the left");
//            telemetry.update();
            MyDriveTrain.encoderDrive(1,-140,-140,-140,-140,2);
            MyDriveTrain.encoderDrive(0.8,-70,70,70,-70,1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1,-25,-25,-25,-25,2);
            sleep(500);
//            if (cubeIn.getDistance(DistanceUnit.MM)>60){
//                MyDriveTrain.encoderDrive(0.7,-20,-20,-20,-20,2);
//                telemetry.addData("CubeDistans:",cubeIn.getDistance(DistanceUnit.MM));
//                telemetry.update();
//            }
//
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8,60,-60,-60,60,1);
            MyDriveTrain.encoderDrive(1,180,180,180,180,2);


        }
        else {
//            telemetry.addLine("You are on the center!");
//            telemetry.update();
            MyDriveTrain.encoderDrive(1,-140,-140,-140,-140,2);
            MyDriveTrain.encoderDrive(0.8,-70,70,70,-70,1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1,-25,-25,-25,-25,2);
            sleep(500);
//            if (cubeIn.getDistance(DistanceUnit.MM)>60){
//                MyDriveTrain.encoderDrive(0.7,-20,-20,-20,-20,2);
//                telemetry.addData("CubeDistans:",cubeIn.getDistance(DistanceUnit.MM));
//                telemetry.update();
//            }

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8,60,-60,-60,60,1);
            MyDriveTrain.encoderDrive(1,180,180,180,180,2);

        }

        MyDriveTrain.RotateP(-90,1,10,0.0208);
        MyIntake.maxOuttake();
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.RotateP(0,1,10,0.0108);
        ParkingMot.setPosition(ParkingMotOut);
        MyDriveTrain.encoderDrive(1,-20,-20,-20,-20,2);

    }

}
