package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "TwoCubeBlue", group = "teamcode")
public class TwoCubeBlue extends basicAuto {

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

        MyDriveTrain.encoderDrive(0.8,-30,30,30,-30 ,1);
        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();
        MyDriveTrain.Rotate(180,0.1,10);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Mikum>2) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            MyDriveTrain.encoderDrive(1,85,85,85,85,2);
//            sleep(500);
            MyDriveTrain.encoderDrive(1,36,36,-36,-36,2);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.7,-80,-80,-80,-80,2);
            if (cubeIn.getDistance(DistanceUnit.MM) > 60){
                MyDriveTrain.encoderDrive(0.7,-20,-20,-20,-20,2);
                telemetry.addData("CubeDistans:",cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
//            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(1,60,60,60,60,2);
//            sleep(500);
            MyDriveTrain.encoderDrive(1,-37,-37,37,37,2);
//            sleep(500);
            MyDriveTrain.encoderDrive(1,70,70,70,70,2);
//            sleep(500);


        }
        else if (Mikum<-2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.encoderDrive(1,43,43,43,43,2);
//            sleep(500);
            MyDriveTrain.encoderDrive(0.8,-90,90,90,-90,1);
//            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1,-25,-25,-25,-25,2);
//            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > 60){
                MyDriveTrain.encoderDrive(0.7,-20,-20,-20,-20,2);
                telemetry.addData("CubeDistans:",cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8,65,-65,-65,65,1);
//            sleep(500);
            MyDriveTrain.encoderDrive(1,120,120,120,120,2);
//            sleep(500);

        }
        else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(1,50,50,50,50,2);
//            sleep(500);
            MyDriveTrain.encoderDrive(0.8,-90,90,90,-90,1);
//            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1,-25,-25,-25,-25,2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > 60){
                MyDriveTrain.encoderDrive(0.7,-20,-20,-20,-20,2);
                telemetry.addData("CubeDistans:",cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8,65,-65,-65,65,1);
//            sleep(500);
            MyDriveTrain.encoderDrive(1,120,120,120,120,2);
//            sleep(500);
        }

        MyDriveTrain.encoderDrive(0.8,60,60,-60,-60,2);
//        sleep(500);
        MyIntake.maxOuttake();
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.encoderDrive(0.8,-55,-55,55,55,2);
//        sleep(500);

        if (Mikum>2) {
//            telemetry.addLine("you're on the right");
//            telemetry.update();
            MyDriveTrain.encoderDrive(1,-117,-117,-117,-117,2);
//            sleep(500);
            MyDriveTrain.encoderDrive(1,-65,65,65,-65,1);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1,-25,-25,-25,-25,2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM)>60){
                MyDriveTrain.encoderDrive(1,-20,-20,-20,-20,2);
                telemetry.addData("CubeDistans:",cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
//            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(1,60,-60,-60,60,1);
//            sleep(500);
            MyDriveTrain.encoderDrive(1,150,150,150,150,2);
//            sleep(500);
        }
        else if (Mikum<-2) {
//            telemetry.addLine("you're on the left");
//            telemetry.update();
            MyDriveTrain.encoderDrive(1,-140,-140,-140,-140,2);
//            sleep(500);
            MyDriveTrain.encoderDrive(0.8,-70,70,70,-70,1);
//            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1,-25,-25,-25,-25,2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM)>60){
                MyDriveTrain.encoderDrive(0.7,-20,-20,-20,-20,2);
                telemetry.addData("CubeDistans:",cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
//            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8,60,-60,-60,60,1);
//            sleep(500);
            MyDriveTrain.encoderDrive(1,180,180,180,180,2);
//            sleep(500);


        }
        else {
//            telemetry.addLine("You are on the center!");
//            telemetry.update();
            MyDriveTrain.encoderDrive(1,-140,-140,-140,-140,2);
//            sleep(500);
            MyDriveTrain.encoderDrive(0.8,-70,70,70,-70,1);
//            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1,-25,-25,-25,-25,2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM)>60){
                MyDriveTrain.encoderDrive(0.7,-20,-20,-20,-20,2);
                telemetry.addData("CubeDistans:",cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
//            sleep(500);
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8,60,-60,-60,60,1);
//            sleep(500);
            MyDriveTrain.encoderDrive(1,180,180,180,180,2);
//            sleep(500);

        }

        MyDriveTrain.encoderDrive(0.8,60,60,-60,-60,2);
//        sleep(500);
        MyIntake.maxOuttake();
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.encoderDrive(1,10,10,10,10,1);
        MyDriveTrain.encoderDrive(1,40,-40,-40,40,1);




    }

}
