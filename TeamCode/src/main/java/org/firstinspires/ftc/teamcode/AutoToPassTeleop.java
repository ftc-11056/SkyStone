package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous(name="AutoToPassTeleop", group="teamcode")
public class AutoToPassTeleop extends basicAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        MyDriveTrain.Rotate(130,0.6,60000);
    }

}
