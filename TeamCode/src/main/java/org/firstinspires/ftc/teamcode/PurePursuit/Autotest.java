package org.firstinspires.ftc.teamcode.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name="Autotest", group="teamcode")
@Disabled
public class Autotest extends Robot {

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

        while (opModeIsActive()) {
            double currentTime = runtime.seconds();
            double odometryRight = MyDriveTrain.RightFront.getCurrentPosition();
            double odometryLeft = MyDriveTrain.LeftFront.getCurrentPosition();
            double odometryHorizental = MyDriveTrain.RightBack.getCurrentPosition();
            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double direction = angles.firstAngle;
            MyOdometry.setAll(odometryRight, odometryLeft, odometryHorizental, direction, currentTime);
            MyPurePursuitGUI.UpdatePowerByRobotPosition(runtime.seconds(), MyOdometry.getPosition(), MyOdometry.getDirection(), MyOdometry.getVelocityX(), MyOdometry.getVelocityY());
            MyPurePursuitGUI.updateGraghic();
        }
    }



}
