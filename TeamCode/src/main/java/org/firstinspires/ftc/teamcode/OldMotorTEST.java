package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="OldMotorTEST", group="teamcode")
public class OldMotorTEST extends LinearOpMode {
    DcMotor yos;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode () throws InterruptedException {

        yos = hardwareMap.get(DcMotor.class, "yos");

        yos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yos.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        runtime.reset();
        yos.setPower(1);

        while(runtime.seconds()<5){
            telemetry.addData("encoder:", yos.getCurrentPosition());
            telemetry.update();
        }
        yos.setPower(0);
        telemetry.addData("encoder:", yos.getCurrentPosition());
        telemetry.update();

        sleep(8000);
    }
}
