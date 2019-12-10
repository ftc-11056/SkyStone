package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "OneCubeBlue", group = "teamcode")
public class OneCubeBlue extends basicAuto {

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
        MyDriveTrain.Rotate(0,0.1,10);
        telemetry.addData("angles", MyDriveTrain.getAngle());
        telemetry.update();
        sleep(2000);
        Mikum = MyVuforiaStone.ConceptVuforiaSkyStoneNavigationWebcam();
        MyDriveTrain.RotateP(-180,1,10,0.0108);
//        Mikum = -3;
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (Mikum < -2) {
            telemetry.addLine("you're on the right");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, 13, 13, 13, 13, 1);
            MyDriveTrain.encoderDrive(0.3, 65, -65, -65, 65, 1);
           // MyDriveTrain.Rotate(-180,0.1,10);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.8, -25, -25, -25, -25, 2);
            sleep(500);
            MyDriveTrain.Verification(cubeIn,cubeNotInMM);

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.9, -52, 52, 52, -52, 1);
            MyDriveTrain.Rotate(-180,0.1,10);
            MyDriveTrain.encoderDrive(0.8, 100, 100, 100, 100, 2);
        }

        else if (Mikum > 2) {
            telemetry.addLine("you're on the left");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, 40, 40, 40, 40, 2);
            MyDriveTrain.encoderDrive(0.3, 71, -71, -71, 71, 1);
            MyDriveTrain.Rotate(-180,0.1,10);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.8, -25, -25, -25, -25, 2);
            sleep(500);
            MyDriveTrain.Verification(cubeIn,cubeNotInMM);

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.5, -52, 52, 52, -52, 1);
            MyDriveTrain.Rotate(-180,0.1,10);
            MyDriveTrain.encoderDrive(1, 85, 85, 85, 85, 2);



        } else {
            telemetry.addLine("You are on the center!");
            telemetry.update();
            MyDriveTrain.encoderDrive(0.8, 50, 50, 50, 50, 2);
            MyDriveTrain.encoderDrive(0.3, 73, -73, -73, 73, 1);
            MyDriveTrain.Rotate(-180,0.1,10);
            MyIntake.maxIntake();
            MyDriveTrain.encoderDrive(0.8, -25, -25, -25, -25, 2);
            sleep(500) ;
            MyDriveTrain.Verification(cubeIn,cubeNotInMM);

            MyIntake.ShutDown();
            MyDriveTrain.encoderDrive(0.5, -52, 52, 52, -52, 1);
            MyDriveTrain.Rotate(-180,0.1,10);
            MyDriveTrain.encoderDrive(1, 70, 70, 70, 70, 2);

        }

        MyDriveTrain.Rotate(-180,0.2,10);
        MyIntake.maxIntake();
        sleep(500);
        MyIntake.ShutDown();
        Output.setPosition(OutputDown);
        MyDriveTrain.encoderDrive(0.8, 110, 110, 110, 110, 2);
        MyDriveTrain.RotateP(-270,1,10,0.0108);
        MyDriveTrain.encoderDrive(0.2,20,20,20,20,2);
        MyDriveTrain.Rotate(-270,0.1,10);
        LeftServo.setPosition(0.15);
        RightServo.setPosition(0.2);
        MyDriveTrain.encoderDrive(0.1,10,10,10,10,2);
        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(500);
        MyDriveTrain.encoderDrive(1,-100,-100,-100,-100,2);
        MyDriveTrain.Rotate(-180,0.5,10);
        MyDriveTrain.encoderDrive(1,30,30,30,30,2);

        MyElevator.ElevateWithEncoder(-450, 0.8, 0.5);
        sleep(500);
        Arm.setPosition(1);
        sleep(700);
        MyElevator.ElevateWithEncoder(0, 0.3, 0.0035);
        sleep(700);
        Output.setPosition(OutputUp);
        sleep(500);
        MyElevator.ElevateWithEncoder(-500, 0.3, 0.5);
        Arm.setPosition(0.135);
        sleep(1000);
        MyElevator.ElevateWithEncoder(0, 0.5, 0.003);

        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);
        MyDriveTrain.Rotate(-180,0.4,10);
        MyDriveTrain.encoderDrive(0.3,30,-30,-30,30,2);
        ParkingMot.setPosition(ParkingMotIn);
        sleep(500);
        MyDriveTrain.encoderDrive(0.5,-43,-43, -43,-43,1);   }

}
