package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.PP.OurPoint;
import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@Autonomous(name = "Autotest", group = "teamcode")
public class Autotest extends Robot {


    public PurePursuitGUI MyPurePursuitGUI;
    public FtcDashboard dashboard;
    private OurPoint StartPosition = new OurPoint(-0.19, -1.61, 180);
    private OurPoint[] Path = {
            new OurPoint(-0.19, -1.61, 180),
            new OurPoint(-0.19, -0.61, 180),
            new OurPoint(-1.19, -0.61, 180)};


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        BuildOdometry(StartPosition);
        //MyPurePursuitGUI = new PurePursuitGUI(Path, MyOdometry.getPosition());

        while(!isStarted()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            dashboard.sendTelemetryPacket(packet);
        }
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            packet = new TelemetryPacket();
            double currentTime = runtime.seconds();
            updateOdometry();
            MyPurePursuitGUI.UpdatePowerByRobotPosition(packet,currentTime, MyOdometry.getPosition(), MyOdometry.getVelocityX(), MyOdometry.getVelocityY());
            if (MyPurePursuitGUI.stop){
                StopAndSavePlace(MyPurePursuitGUI, dashboard);
                //MyDriveTrain.stop();
                packet.addLine("the end");
                dashboard.sendTelemetryPacket(packet);
                break;
            }
            else {
                MyDriveTrain.arcade(MyPurePursuitGUI.getYpower(), MyPurePursuitGUI.getXpower(), MyPurePursuitGUI.getCpower());
            }
            MyPurePursuitGUI.updateGraghic(packet);
            dashboard.sendTelemetryPacket(packet);
        }



    }


}
