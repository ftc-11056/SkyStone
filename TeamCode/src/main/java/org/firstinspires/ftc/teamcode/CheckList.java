package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "CheckList", group = "teamcode")
public class CheckList extends basicAuto {

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

        MyIntake.maxIntake();
        MyElevator.ElevateWithEncoder(-500, 0.3, 0.5);
        sleep(1000);
        MyIntake.ShutDown();
        Arm.setPosition(1);
        sleep(1000);
        MyElevator.ElevateWithEncoder(0, 0.1, 0.003);

        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(1000);
        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);
        sleep(1000);

        Output.setPosition(OutputDown);
        sleep(1000);
        Output.setPosition(OutputUp);
        sleep(1000);

        ParkingMot.setPosition(ParkingMotIn);
        sleep(1000);
        ParkingMot.setPosition(ParkingMotOut);
        sleep(1000);


    }

}
