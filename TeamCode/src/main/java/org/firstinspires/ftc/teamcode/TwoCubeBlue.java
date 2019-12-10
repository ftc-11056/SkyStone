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
    public double cubeNotInMM = 150;

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
        MyDriveTrain.RotateP(-180,1,10,0.0108);
        MyDriveTrain.Rotate(-180,0.1,10);

//        Mikum = -3;

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Mikum < -2
        ) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, 35, -35, -35, 35, 1);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(1, 70, 70, 70, 70, 2);
            MyDriveTrain.RotateP(-135, 0.7, 10, 0.0208);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.2, -60, -60, -60, -60, 2);
            sleep(500);
            MyDriveTrain.Verification(cubeIn,cubeNotInMM);

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(1, 55, 55, 55, 55, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(1, 70, 70, 70, 70, 2);


        } else if (Mikum > 2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, 37, 37, 37, 37, 2);
            MyDriveTrain.encoderDrive(0.3, 70, -70, -70, 70, 1);
            MyDriveTrain.Rotate(-180,0.1,10);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.8, -25, -25, -25, -25, 2);
            sleep(500);
            MyDriveTrain.Verification(cubeIn,cubeNotInMM);

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, 10, 10, 10, 10, 2);
            MyDriveTrain.encoderDrive(0.5, -50, 50, 50, -50, 1);
            MyDriveTrain.Rotate(-180,0.1,10);
            MyDriveTrain.encoderDrive(1, 70, 70, 70, 70, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(1, 60, 60, 60, 60, 2);



        } else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, 50, 50, 50, 50, 2);
            MyDriveTrain.encoderDrive(0.3, 70, -70, -70, 70, 1);
            MyDriveTrain.Rotate(-180,0.1,10);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.8, -25, -25, -25, -25, 2);
            sleep(500) ;
            MyDriveTrain.Verification(cubeIn,cubeNotInMM);

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.2, 10, 10, 10, 10, 2);
            MyDriveTrain.encoderDrive(0.5, -50, 50, 50, -50, 1);
            MyDriveTrain.Rotate(-180,0.1,10);
            MyDriveTrain.encoderDrive(1, 60, 50, 50, 50, 2);
            MyDriveTrain.Rotate(-180,0.1,10);
            MyDriveTrain.encoderDrive(1, 50, 50, 50, 50, 2);

        }

        MyDriveTrain.RotateP(-90, 1, 10, 0.0208);
        MyIntake.maxOuttake();
        sleep(500);
        MyIntake.ShutDown();
        MyDriveTrain.RotateP(-180, 0.8, 10, 0.0108);
        MyDriveTrain.Rotate(-180, 0.1, 10);



        if (Mikum > 2) {
//            telemetry.addLine("you're on the right");
//            telemetry.update();
            MyDriveTrain.encoderDrive(1, -60, -60, -60, -60, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(1, -55, -55, -55, -55, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(1, 55, -55, -55, 55, 1);
            sleep(500);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(1, -25, -25, -25, -25, 2);
            sleep(500);
            MyDriveTrain.Verification(cubeIn,cubeNotInMM);

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, -50, 50, 50, -50, 1);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(1, 90, 90, 90, 90, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(1, 60, 60, 60, 60, 2);

        } else if (Mikum < -2) {
//            telemetry.addLine("you're on the left");
//            telemetry.update();
            MyDriveTrain.encoderDrive(0.7, -80, -80, -80, -80, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, -70, -70, -70, -70, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, 53, -53, -53,53,71);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.2, -25, -25, -25, -25, 2);
            sleep(500);
            MyDriveTrain.Verification(cubeIn,cubeNotInMM);

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.3, 10, 10, 10, 10, 2);

            MyDriveTrain.encoderDrive(0.8, -55, 55, 55, -55, 1);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, 90, 90, 90, 90, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, 90, 90, 90, 90, 2);



        } else {
//            telemetry.addLine("You are on the center!");
//            telemetry.update();
            MyDriveTrain.encoderDrive(0.7, -63, -63, -63, -63, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, -60, -60, -60, -60, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(0.6, 40, -40, -40, 40, 1);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.3, -28, -28, -28, -28, 2);
            sleep(500);
            MyDriveTrain.Verification(cubeIn,cubeNotInMM);

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.8, -60, 60, 60, -60, 1);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, 90, 90, 90, 90, 2);
            MyDriveTrain.Rotate(-180, 0.1, 10);
            MyDriveTrain.encoderDrive(0.7, 80, 80, 80, 80, 2);



        }

        MyDriveTrain.RotateP(-90,1,10,0.0108);
        MyIntake.maxOuttake();
        ParkingMot.setPosition(ParkingMotIn);
        sleep(1000);
        MyIntake.ShutDown();
        MyDriveTrain.RotateP(-180, 1, 1.5, 0.0108);

        MyDriveTrain.encoderDrive(1, -20, -20, -20, -20, 2);

    }

}
