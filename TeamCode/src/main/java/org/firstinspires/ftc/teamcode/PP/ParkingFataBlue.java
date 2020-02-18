package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PP.PurePursuitGUI;
import org.firstinspires.ftc.teamcode.PP.OurPoint;
import org.firstinspires.ftc.teamcode.RobotCustomade;
import org.firstinspires.ftc.teamcode.basicAutoCustumade;

@Autonomous(name = "ParkingFataBlue", group = "teamcode")
@Disabled
public class ParkingFataBlue extends RobotCustomade {

    public PurePursuitGUI MyPurePursuitGUI;
    private OurPoint StartPosition = new OurPoint(-1.566, 0.368, 0);
    public TelemetryPacket packet = null;
    private Boolean isRun = true;

    private static OurPoint[] Path1 = {
            new OurPoint(-1.566, 0.368, 0),
            new OurPoint(-1.55, 1.48, 0)};
    private static double tolerancePath1 = 10;
    private static double KcPath1 = 4.5;
    private static double MaxVelocityPath1 = 1.5;
    private static double turnSpeedPath1 = 0.7;
    private static boolean frontPath1 = true;

    private static OurPoint[] Path2 = {
            new OurPoint(-1.55, 1.48, 0),
            new OurPoint(-1.55, 0.1, 0)};
    private static double tolerancePath2 = 10;
    private static double KcPath2 = 4.5;
    private static double MaxVelocityPath2 = 1.5;
    private static double turnSpeedPath2 = 0.7;
    private static boolean frontPath2 = false;
    //public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //  dashboard = FtcDashboard.getInstance();
        //packet = new TelemetryPacket();
        BuildOdometry(StartPosition);

        waitForStart();
        runtime.reset();
        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Path1, MyOdometry.getPosition(), tolerancePath1, KcPath1, MaxVelocityPath1, turnSpeedPath1, frontPath1);
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }
        sleep(8000);

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Path2, MyOdometry.getPosition(), tolerancePath2, KcPath2, MaxVelocityPath2, turnSpeedPath2, frontPath2);
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }
    }

    public boolean purePesuitRun() {
        packet = new TelemetryPacket();
        double currentTime = runtime.seconds();
        updateOdometry();
        MyPurePursuitGUI.UpdatePowerByRobotPosition(packet, currentTime, MyOdometry.getPosition(), MyOdometry.getVelocityX(), MyOdometry.getVelocityY());
        if (MyPurePursuitGUI.stop) {
            MyDriveTrain.stop();
            packet.addLine("the end");
            return false;
        } else {
            MyDriveTrain.arcade(MyPurePursuitGUI.getYpower(), MyPurePursuitGUI.getXpower(), MyPurePursuitGUI.getCpower());
        }
        return true;
    }
}