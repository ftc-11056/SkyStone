package org.firstinspires.ftc.teamcode.Autonomuses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.RobotCustomade;

@Autonomous(name = "ParkingAutonomaus", group = "teamcode")
public class ParkingAutonomaus extends RobotCustomade {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        sleep(25000);
        Arm.setPosition(ArmOpen);
        sleep(2000);
    }

}
