package org.firstinspires.ftc.teamcode;

import android.support.v4.app.ServiceCompat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.basicAuto;
import org.firstinspires.ftc.teamcode.basicAutoCustumade;


@Autonomous(name = "CheckList", group = "teamcode")
public class CheckList extends basicAutoCustumade {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        runtime.reset();

        LF.setTargetPosition(400);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setPower(1);
        while (LF.isBusy()) {
            telemetry.addData("LFencoder: ", LF.getCurrentPosition());
            telemetry.update();
        }
        LF.setPower(0);

        sleep(1000);

        RF.setTargetPosition(400);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setPower(1);
        while (RF.isBusy()) {
            telemetry.addData("LFencoder: ", RF.getCurrentPosition());
            telemetry.update();
        }
        RF.setPower(0);

        sleep(1000);

        LB.setTargetPosition(400);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setPower(1);
        while (LB.isBusy()) {
            telemetry.addData("LFencoder: ", LB.getCurrentPosition());
            telemetry.update();
        }
        LB.setPower(0);

        sleep(1000);

        RB.setTargetPosition(400);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setPower(1);
        while (RB.isBusy()) {
            telemetry.addData("LFencoder: ", RB.getCurrentPosition());
            telemetry.update();
        }
        RB.setPower(0);
        telemetry.addData("Angles", MyDriveTrain.getAngle());
        sleep(1500);
        telemetry.update();

        MyDriveTrain.Rotate(90,1,10);

        MyElevator.ElevateWithEncoder(-500, 0.3, 0.5);

        sleep(1000);
        Capstone.setPosition(CapstoneDown);
        Capstone.setPosition(CapstoneUp);
        sleep(1000);

        MyElevator.ElevateWithEncoder(0, 0.1, 0.003);

        MyIntake.maxIntake();
        sleep(1000);
        MyIntake.maxOuttake();

        MyDriveTrain.SetPower(0.8,0.8,0.8,0.8);
        sleep(1000);
        MyDriveTrain.SetPower(0,0,0,0);
        telemetry.addData("Odometry Y", MyOdometry.getPosition().getY());
        telemetry.update();
        sleep(4000);

        MyDriveTrain.SetPower(-0.8,0.8,0.8,-0.8);
        sleep(1000);
        MyDriveTrain.SetPower(0,0,0,0);
        telemetry.addData("Odometry X", MyOdometry.getPosition().getX());
        telemetry.update();

        sleep(4000);

        Arm.setPosition(ArmOpen);
        Arm.setPosition(ArmClose);

        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(1000);
        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);

        sleep(1000);

        Output.setPosition(OutputClose);
        sleep(1000);
        Output.setPosition(OutputOpen);
        sleep(1000);

        ParkingMot.setPosition(ParkingMotIn);
        sleep(1000);
        ParkingMot.setPosition(ParkingMotOut);
        sleep(1000);

        telemetry.addData("down Magnet", downMagnetElevator.getState());
        telemetry.addData("CubeIn", cubeIn.getDistance(DistanceUnit.MM));
        telemetry.update();
        sleep(3000);

        telemetry.addData("left touch", LeftTouch.getState());
        telemetry.addData("right touch", RightTouch.getState());
        telemetry.update();
        sleep(2000);
    }
}