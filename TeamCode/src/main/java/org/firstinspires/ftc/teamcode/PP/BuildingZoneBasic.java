package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name="BuildingZoneBasic", group="teamcode")
@Disabled
public class BuildingZoneBasic extends Robot {

    /* Declare OpMode members. */
    private org.firstinspires.ftc.teamcode.PP.PurePursuitGUI MyPurePursuitGUI;
    private FtcDashboard dashboard;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

//        MyDriveTrain.startAndResetEncoders();

        waitForStart();

//        MyDriveTrain.encoderDrive(0.1,30,30,30,30);

        telemetry.addData("LeftFront Encoders", MyDriveTrain.LeftFront.getCurrentPosition());
        telemetry.addData("LeftBack Encoders", MyDriveTrain.LeftBack.getCurrentPosition());
        telemetry.addData("RightFront Encoders", MyDriveTrain.RightFront.getCurrentPosition());
        telemetry.addData("RightBack Encoders", MyDriveTrain.RightBack.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("LeftFront Power", MyDriveTrain.LeftFront.getPower());
        telemetry.addData("LeftBack Power", MyDriveTrain.LeftBack.getPower());
        telemetry.addData("RightFront Power", MyDriveTrain.RightFront.getPower());
        telemetry.addData("RightBack Power", MyDriveTrain.RightBack.getPower());
        telemetry.update();

        sleep(110563211);

        }
    }








