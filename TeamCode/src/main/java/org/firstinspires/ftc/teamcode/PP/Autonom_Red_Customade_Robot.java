package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCustomade;


@Autonomous(name = "Autonom_Red_Customade_Robot", group = "teamcode")
public class Autonom_Red_Customade_Robot extends RobotCustomade /*basicAutoCustumade*/ {

    public PurePursuitGUI MyPurePursuitGUI;
    public FtcDashboard dashboard;
    private OurPoint StartPosition = new OurPoint(1.566, -0.8325, 90);
    public TelemetryPacket packet = null;
    private Boolean isRun = true;
    public double deltaFromFlatAngle = 0;
    private boolean ElevateorBusy = true;
    private double petchX = 1;
    private double pechY = 1;
    private double distanceToCenter;
    private double factor = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        BuildOdometry(StartPosition);

//        Mikum = skystoneDetector.getScreenPosition().y;

        Path[] Paths = Paths_Library_Red.RightPaths;


        MyPurePursuitGUI = new PurePursuitGUI(Paths[2].getWayPoints(), MyOdometry.getPosition(), Paths[2].getTolerance(), Paths[2].getKc(), Paths[2].getMaxVelocity(), Paths[2].getTurnSpeed(), Paths[2].isFront());
            while (!isStarted()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            dashboard.sendTelemetryPacket(packet);
//            Mikum = skystoneDetector.getScreenPosition().y;
//            telemetry.addData("skystone", Mikum);
//            telemetry.update();
        }
        runtime.reset();
        waitForStart();
//        webcam.closeCameraDevice();

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[0].getWayPoints(), MyOdometry.getPosition(), Paths[0].getTolerance(), Paths[0].getKc(), Paths[0].getMaxVelocity(), Paths[0].getTurnSpeed(), Paths[0].isFront());
        MyPurePursuitGUI.setKv(1);
        distanceToCenter = 1;
        while (opModeIsActive() && isRun && distanceToCenter >= 0.2) {
            isRun = purePesuitRun();
            distanceToCenter = MyMath.distance(MyOdometry.getPosition(), Paths[0].getWayPoints()[Paths[0].getWayPoints().length-1]);
            if (MyPurePursuitGUI.findClosetPointIndex() >= 10) {
                MyPurePursuitGUI.setKa(0);
                //MyPurePursuitGUI.setKv(0.5);
                factor = 0.4;
                MyIntake.maxIntake();
            }
        }

        //MyDriveTrain.Verification(cubeIn,cubeNotInMM,packet,dashboard);

        factor = 1;
        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[1].getWayPoints(), MyOdometry.getPosition(), Paths[1].getTolerance(), Paths[1].getKc(), Paths[1].getMaxVelocity(), Paths[1].getTurnSpeed(), Paths[1].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() >= 20) {
                MyIntake.ShutDown();
            }

            if (MyPurePursuitGUI.findClosetPointIndex() >= 33) {
                LeftServo.setPosition(LeftServoMiddle);
                RightServo.setPosition(RightServoMiddle);
                MyPurePursuitGUI.setKv(0.4);
            }
        }

//        MyDriveTrain.TouchFoundation(LeftTouch,RightTouch);
        Output.setPosition(OutputClose);
        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(400);

        distanceToCenter = 1;
        isRun = true;
        PlacingStoneTime = runtime.seconds();
        MyPurePursuitGUI = new PurePursuitGUI(Paths[2].getWayPoints(), MyOdometry.getPosition(), Paths[2].getTolerance(), Paths[2].getKc(), Paths[2].getMaxVelocity(), Paths[2].getTurnSpeed(), Paths[2].isFront());
        while (opModeIsActive() && isRun && distanceToCenter >= 0.14) {
            isRun = purePesuitRun();
            distanceToCenter = MyMath.distance(MyOdometry.getPosition(), Paths[2].getWayPoints()[Paths[2].getWayPoints().length-1]);
            if (MyPurePursuitGUI.findClosetPointIndex() <= 10){
                MyPurePursuitGUI.setTurnSpeed(0);
                petchX = 0;
            }
            else if (MyPurePursuitGUI.findClosetPointIndex() <= 30) {
                packet.addLine("switch");
                petchX = 1;
                MyPurePursuitGUI.setTurnSpeed(2.5);
            }
          if (MyPurePursuitGUI.findClosetPointIndex() <= 25) {
             PlacingStoneWhitTime(packet);
          }

            deltaFromFlatAngle = Math.abs(MyOdometry.getDirection() - Math.toRadians(270));
            if (MyPurePursuitGUI.findClosetPointIndex() >= 25){
                LeftServo.setPosition(LeftServoUp);
                RightServo.setPosition(RightServoUp);
                //MyPurePursuitGUI.setKv(0.4);
                MyPurePursuitGUI.setTurnSpeed(1);
            }
            if (MyPurePursuitGUI.findClosetPointIndex() >= 52) {
                Output.setPosition(OutputOpen);
                MyIntake.maxIntake();
            }
            if(MyPurePursuitGUI.findClosetPointIndex() >= 60){
                factor = 0.4;
            }
        }
        factor = 1;
        MyDriveTrain.Verification(cubeIn,cubeNotInMM,packet,dashboard);

        isRun = true;
        ElevateorBusy = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[3].getWayPoints(), MyOdometry.getPosition(), Paths[3].getTolerance(), Paths[3].getKc(), Paths[3].getMaxVelocity(), Paths[3].getTurnSpeed(), Paths[3].isFront());
        while (opModeIsActive() && (isRun)) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() >= 15) {
                MyIntake.ShutDown();
                Output.setPosition(OutputClose);
            }
            if(MyPurePursuitGUI.findClosetPointIndex() == 39){
                PlacingStoneTime = runtime.seconds();
            }
            if(MyPurePursuitGUI.findClosetPointIndex() >= 40){
                ElevateorBusy = PlacingStoneWhitTime(packet);
            }
        }


        //        TODO Parking
        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[4].getWayPoints(), MyOdometry.getPosition(), Paths[4].getTolerance(), Paths[4].getKc(), Paths[4].getMaxVelocity(), Paths[4].getTurnSpeed(), Paths[4].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }


//        TODO Telemetry
        while (!isStopRequested()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            packet.put("isRun", isRun.toString());
            dashboard.sendTelemetryPacket(packet);
        }

    }

    private void LocalUpdateGraphic() {
        packet.put("distanceToCenter", distanceToCenter);
        packet.put("distanceToCube", cubeIn.getDistance(DistanceUnit.MM));
    }

    public boolean purePesuitRun() {
        packet = new TelemetryPacket();
        double currentTime = runtime.seconds();
        updateOdometry();
        MyPurePursuitGUI.UpdatePowerByRobotPosition(packet, currentTime, MyOdometry.getPosition(), MyOdometry.getVelocityX(), MyOdometry.getVelocityY());
        if (MyPurePursuitGUI.stop) {
            MyDriveTrain.stop();
            packet.addLine("the end");
            dashboard.sendTelemetryPacket(packet);
            return false;
        } else {
            MyDriveTrain.arcade(factor*MyPurePursuitGUI.getYpower(), factor* petchX *MyPurePursuitGUI.getXpower(), factor*MyPurePursuitGUI.getCpower());
        }
        MyPurePursuitGUI.updateGraghic(packet);
        LocalUpdateGraphic();
        dashboard.sendTelemetryPacket(packet);
        return true;
    }

}


