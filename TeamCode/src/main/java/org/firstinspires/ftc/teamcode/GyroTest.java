package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "GyroTest", group = "teamcode")
@Disabled
public class GyroTest extends Robot {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        MyDriveTrain.Rotate(-90,0.4,100);
        sleep(1500);
        MyDriveTrain.Rotate(180,0.4,10);
        sleep(1500);
        MyDriveTrain.Rotate(-90,0.4,10);
        telemetry.addData("bla",MyDriveTrain.getAngle());
        sleep(2000);
        telemetry.update();


    }
}