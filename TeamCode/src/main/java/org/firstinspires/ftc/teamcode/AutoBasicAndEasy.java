package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="AutoBasicAndEasy", group="teamcode")
public class AutoBasicAndEasy extends Robot{

    /* Declare OpMode members. */
    private PurePursuitGUI MyPurePursuitGUI;
    private FtcDashboard dashboard;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        dashboard = FtcDashboard.getInstance();
        MyPurePursuitGUI = new PurePursuitGUI(dashboard);
        MyOdometry = MyPurePursuitGUI.buildOdometry();

        waitForStart();
        MyDriveTrain.encoderDrive(0.3,10,10,10,10);
        sleep(1000);
        MyDriveTrain.encoderDrive(0.3,40,-40,-40,40);
    }



}
