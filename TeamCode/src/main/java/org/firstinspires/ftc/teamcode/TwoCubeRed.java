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

        MyDriveTrain.encoderDrive(0.8, -30, 30, 30, -30, 1);
        MyDriveTrain.Rotate(0, 0.1, 10);
        Output.setPosition(OutputDown);
        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();
        Mikum = -3;

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Mikum > 2) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, -35, 35, 35, -35, 1);
            MyDriveTrain.encoderDrive(1, 70, 70, 70, 70, 2);
            MyDriveTrain.RotateP(-35, 0.7, 10, 0.0108);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.2, -60, -60, -60, -60, 2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > cubeNotInMM) {
                MyDriveTrain.encoderDrive(0.2, -20, -20, -20, -20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
                MyDriveTrain.encoderDrive(0.2, 20, 20, 20, 20, 2);

            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(1, 50, 50, 50, 50, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(1, 80, 80, 80, 80, 2);


        } else if (Mikum < -2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, 18, 18, 18, 18, 2);
            MyDriveTrain.encoderDrive(0.8, -86, 86, 86, -86, 1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.8, -25, -25, -25, -25, 2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > cubeNotInMM) {
                MyDriveTrain.encoderDrive(0.2, -20, -20, -20, -20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
                MyDriveTrain.encoderDrive(0.2, 20, 20, 20, 20, 2);

            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 55, -55, -55, 55, 1);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, 65, 65, 65, 65, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, 65, 65, 65, 65, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);


        } else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(1, 30, 30, 30, 30, 2);
            MyDriveTrain.encoderDrive(0.8, -84, 84, 84, -84, 1);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.2, -25, -25, -25, -25, 2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > cubeNotInMM) {
                MyDriveTrain.encoderDrive(0.2, -20, -20, -20, -20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
                MyDriveTrain.encoderDrive(0.2, 20, 20, 20, 20, 2);

            }
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.2, 10, 10, 10, 10, 2);
            MyDriveTrain.encoderDrive(0.8, 50, -50, -50, 50, 1);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(1, 120, 120, 120, 120, 2);

        }

        MyDriveTrain.RotateP(-90, 1, 10, 0.0208);
        MyIntake.maxOuttake();
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.RotateP(0, 1, 10, 0.0108);


        if (Mikum > 2) {
//            telemetry.addLine("you're on the right");
//            telemetry.update();
            MyDriveTrain.encoderDrive(1, -73, -73, -73, -73, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(1, -78, -78, -78, -78, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(1, -62, 62, 62, -62, 1);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1, -25, -25, -25, -25, 2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > cubeNotInMM) {
                MyDriveTrain.encoderDrive(0.2, -20, -20, -20, -20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
                MyDriveTrain.encoderDrive(0.2, 20, 20, 20, 20, 2);

            }

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 45, -45, -45, 45, 1);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(1, 90, 90, 90, 90, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(1, 75, 75, 75, 75, 2);

        } else if (Mikum < -2) {
//            telemetry.addLine("you're on the left");
//            telemetry.update();
            MyDriveTrain.encoderDrive(0.7, -80, -80, -80, -80, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, -83, -83, -83, -83, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(0.8, -70, 70, 70, -70, 1);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1, -25, -25, -25, -25, 2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > cubeNotInMM) {
                MyDriveTrain.encoderDrive(0.2, -20, -20, -20, -20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
                MyDriveTrain.encoderDrive(0.2, 20, 20, 20, 20, 2);

            }
//
            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 60, -60, -60, 60, 1);
            MyDriveTrain.encoderDrive(0.7, 110, 110, 110, 110, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, 110, 110, 110, 110, 2);



        } else {
//            telemetry.addLine("You are on the center!");
//            telemetry.update();
            MyDriveTrain.encoderDrive(0.7, -73, -73, -73, -73, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, -77, -77, -77, -77, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(0.8, -65, 65, 65, -65, 1);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.3, -28, -28, -28, -28, 2);
            sleep(500);
            if (cubeIn.getDistance(DistanceUnit.MM) > cubeNotInMM) {
                MyDriveTrain.encoderDrive(0.2, -20, -20, -20, -20, 2);
                telemetry.addData("CubeDistans:", cubeIn.getDistance(DistanceUnit.MM));
                telemetry.update();
                MyDriveTrain.encoderDrive(0.2, 20, 20, 20, 20, 2);

            }

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 50, -50, -50, 50, 1);
            MyDriveTrain.Rotate(0, 0.1, 10);

            MyDriveTrain.encoderDrive(0.7, 90, 90, 90, 90, 2);
            MyDriveTrain.Rotate(0, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, 100, 100, 100, 100, 2);



        }

        MyDriveTrain.RotateP(-90, 1, 10, 0.0208);
        MyIntake.maxOuttake();
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.RotateP(0, 1, 10, 0.0108);
        ParkingMot.setPosition(ParkingMotIn);
        sleep(1500);
        MyDriveTrain.encoderDrive(1, -40, -40, -40, -40, 2);

    }

}
