package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BuildingZoneBasic", group="teamcode")
public class BuildingZoneBasic extends Robot{

    /* Declare OpMode members. */
    private PurePursuitGUI MyPurePursuitGUI;
    private FtcDashboard dashboard;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        MyDriveTrain.startAndResetEncoders();

        waitForStart();

        MyDriveTrain.encoderDrive(0.1,-100,-100,-100,-100);/* go to the front */



        }
        }








