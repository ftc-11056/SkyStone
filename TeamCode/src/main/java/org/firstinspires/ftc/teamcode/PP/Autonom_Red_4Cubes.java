package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCustomade;
import org.firstinspires.ftc.teamcode.basicAutoCustumade;


@Autonomous(name = "Autonom_Red_4Cubes", group = "teamcode")
public class Autonom_Red_4Cubes extends RobotCustomade {

    public PurePursuitGUI MyPurePursuitGUI;
    private OurPoint StartPosition = new OurPoint(1.566, -0.8325, 180);
    private Boolean isRun = true;
    public double deltaFromFlatAngle = 0;
    private boolean ElevateorBusy = true;
    private double petchX = 1;
    private double pechY = 1;
    private double distanceToCenter;
    private double factor = 1;
    private double Cpower = 1;
    private boolean IMUTurn = false;
    Path[] Paths = Paths_Library_Red_4Cubes.LeftPaths;
    private boolean isCubeIn = false;
    private int PointFromEnd = 0;
    private double changeX = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        BuildOdometry(StartPosition);
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//        Mikum = skystoneDetector.getScreenPosition().y;

        int numOfCheck = 4;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[numOfCheck].getWayPoints(), MyOdometry.getPosition(), Paths[numOfCheck].getTolerance(), Paths[numOfCheck].getKc(), Paths[numOfCheck].getMaxVelocity(), Paths[numOfCheck].getTurnSpeed(), Paths[numOfCheck].isFront());
        while (!isStarted()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            dashboard.sendTelemetryPacket(packet);
//            Mikum = skystoneDetector.getScreenPosition().y;
//            telemetry.addData("skystone", Mikum);
//            telemetry.update();
        }

        /*if(Mikum <= 95){
            Paths = Paths_Library_Red_4Cubes.LeftPaths;
        }
        else if (Mikum >= 180){
            Paths = Paths_Library_Red_4Cubes.RightPaths;
        }
        else {
            Paths = Paths_Library_Red_4Cubes.CenterPaths;
        }*/

        runtime.reset();
//        webcam.closeCameraDevice();
        waitForStart();

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[0].getWayPoints(), MyOdometry.getPosition(), Paths[0].getTolerance(), Paths[0].getKc(), Paths[0].getMaxVelocity(), Paths[0].getTurnSpeed(), Paths[0].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[1].getWayPoints(), MyOdometry.getPosition(), Paths[1].getTolerance(), Paths[1].getKc(), Paths[1].getMaxVelocity(), Paths[1].getTurnSpeed(), Paths[1].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[2].getWayPoints(), MyOdometry.getPosition(), Paths[2].getTolerance(), Paths[2].getKc(), Paths[2].getMaxVelocity(), Paths[2].getTurnSpeed(), Paths[2].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[3].getWayPoints(), MyOdometry.getPosition(), Paths[3].getTolerance(), Paths[3].getKc(), Paths[3].getMaxVelocity(), Paths[3].getTurnSpeed(), Paths[3].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[4].getWayPoints(), MyOdometry.getPosition(), Paths[4].getTolerance(), Paths[4].getKc(), Paths[4].getMaxVelocity(), Paths[4].getTurnSpeed(), Paths[4].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[5].getWayPoints(), MyOdometry.getPosition(), Paths[5].getTolerance(), Paths[5].getKc(), Paths[5].getMaxVelocity(), Paths[5].getTurnSpeed(), Paths[5].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[6].getWayPoints(), MyOdometry.getPosition(), Paths[6].getTolerance(), Paths[6].getKc(), Paths[6].getMaxVelocity(), Paths[6].getTurnSpeed(), Paths[6].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[7].getWayPoints(), MyOdometry.getPosition(), Paths[7].getTolerance(), Paths[7].getKc(), Paths[7].getMaxVelocity(), Paths[7].getTurnSpeed(), Paths[7].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[8].getWayPoints(), MyOdometry.getPosition(), Paths[8].getTolerance(), Paths[8].getKc(), Paths[8].getMaxVelocity(), Paths[8].getTurnSpeed(), Paths[8].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }

        //        TODO Telemetry
        while (opModeIsActive()&&!isStopRequested()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            packet.put("isRun", isRun.toString());
            dashboard.sendTelemetryPacket(packet);
        }

    }

    private void FixIntakeByPoint(int index){
        boolean finishFixIntake = false;
        if(MyPurePursuitGUI.findClosetPointIndex() >= index){
            finishFixIntake = IntakeFixing();
        }
        if(finishFixIntake){
            Output.setPosition(OutputClose);
        }
    }

    private void LocalUpdateGraphic() {
        packet.put("PointFromEnd", PointFromEnd);
        packet.put("changeX", changeX);
        packet.put("run time", runtime.seconds());
    }

    public boolean purePesuitRun() {
        packet = new TelemetryPacket();
        double currentTime = runtime.seconds();
        updateOdometry();
        MyPurePursuitGUI.UpdatePowerByRobotPosition(packet, currentTime, MyOdometry.getPosition(), MyOdometry.getVelocityX(), MyOdometry.getVelocityY());
        if(!IMUTurn){
            Cpower = factor*MyPurePursuitGUI.getCpower();
        }
        if (MyPurePursuitGUI.stop) {
            MyDriveTrain.stop();
            packet.addLine("the end");
            dashboard.sendTelemetryPacket(packet);
            return false;
        }
        else {
            MyDriveTrain.arcade(factor*MyPurePursuitGUI.getYpower(), factor* petchX *MyPurePursuitGUI.getXpower(), Cpower);
        }
        MyPurePursuitGUI.updateGraghic(packet);
        LocalUpdateGraphic();
        dashboard.sendTelemetryPacket(packet);
        return true;
    }

}
