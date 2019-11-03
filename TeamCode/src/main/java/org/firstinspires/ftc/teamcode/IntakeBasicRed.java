package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous(name = "IntakeBasicRed", group = "teamcode")
public class IntakeBasicRed extends Robot {

    /* Declare OpMode members. */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Angles:", MyDriveTrain.getAngle());
        telemetry.update();
        MyIntake.maxIntake();
        MyDriveTrain.encoderDrive(0.3,-25,-25,-25,-25,2);
        sleep(500);
        MyIntake.ShutDown();
        MyDriveTrain.encoderDrive(0.3,15,15,15,15,2);
        MyDriveTrain.encoderDrive(0.5,-65,65,65,-65,1);
        MyDriveTrain.encoderDrive(0.3,-15,-15,-15,-15,2);
        MyIntake.move(-1,-1);
        MyIntake.ShutDown();
        MyDriveTrain.encoderDrive(0.3,15,15,15,15,2);
        MyDriveTrain.encoderDrive(0.5,65,-65,-65,65,1);




    }

}
