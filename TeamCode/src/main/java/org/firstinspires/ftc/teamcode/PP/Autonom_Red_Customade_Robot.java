package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotCustomade;


@Autonomous(name = "Autonom_Red_Customade_Robot", group = "teamcode")
public class Autonom_Red_Customade_Robot extends RobotCustomade {

    public PurePursuitGUI MyPurePursuitGUI;
    public FtcDashboard dashboard;
    private OurPoint StartPosition = new OurPoint(1.566, -0.8325, 90);
    public TelemetryPacket packet = null;
    private Boolean isRun = true;
    public double deltaFromFlatAngle = 0;
    private boolean ElevateorBusy = true;
    private double petch = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        BuildOdometry(StartPosition);


        Path[] Paths = Paths_Library.LeftPaths;


        MyPurePursuitGUI = new PurePursuitGUI(Paths[2].getWayPoints(), MyOdometry.getPosition(), Paths[2].getTolerance(), Paths[2].getKc(), Paths[2].getMaxVelocity(), Paths[2].getTurnSpeed(), Paths[2].isFront());
        while (!isStarted()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            dashboard.sendTelemetryPacket(packet);
        }
        runtime.reset();
        waitForStart();


        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[0].getWayPoints(), MyOdometry.getPosition(), Paths[0].getTolerance(), Paths[0].getKc(), Paths[0].getMaxVelocity(), Paths[0].getTurnSpeed(), Paths[0].isFront());
        MyPurePursuitGUI.setKv(1);
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() >= 15) {
                MyPurePursuitGUI.setKa(0);
                MyPurePursuitGUI.setKv(0.6);
                MyIntake.maxIntake();
            }
        }


        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[1].getWayPoints(), MyOdometry.getPosition(), Paths[1].getTolerance(), Paths[1].getKc(), Paths[1].getMaxVelocity(), Paths[1].getTurnSpeed(), Paths[1].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() >= 4) {
                MyIntake.ShutDown();
            }

            if (MyPurePursuitGUI.findClosetPointIndex() >= 37) {
                LeftServo.setPosition(LeftServoMiddle);
                RightServo.setPosition(RightServoMiddle);
                MyPurePursuitGUI.setKv(0.45);
            }

        }

        Output.setPosition(OutputClose);
        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(1000);
/*

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[2].getWayPoints(), MyOdometry.getPosition(), Paths[2].getTolerance(), Paths[2].getKc(), Paths[2].getMaxVelocity(), Paths[2].getTurnSpeed(), Paths[2].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() <= 10) {
                MyPurePursuitGUI.setTurnSpeed(0);
                petch = 0;
            }
            else if (MyPurePursuitGUI.findClosetPointIndex() <= 30) {
                petch = 1;
                packet.addLine("switch");
                MyPurePursuitGUI.setTurnSpeed(2.5);
            }

            setPointIndexStartElavator(0);
//          if (MyPurePursuitGUI.findClosetPointIndex() >= 0 && MyPurePursuitGUI.findClosetPointIndex() <= 45) {

//          }

            deltaFromFlatAngle = Math.abs(MyOdometry.getDirection() - Math.toRadians(270));
            if (MyPurePursuitGUI.findClosetPointIndex() >= 30){
                LeftServo.setPosition(LeftServoUp);
                RightServo.setPosition(RightServoUp);
                MyPurePursuitGUI.setKv(0.5);
                MyPurePursuitGUI.setTurnSpeed(0.5);
            }

            if (MyPurePursuitGUI.findClosetPointIndex() >= 50) {
                Output.setPosition(OutputOpen);
                MyIntake.maxIntake();
            }
        }


        isRun = true;
        ElevateorBusy = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[3].getWayPoints(), MyOdometry.getPosition(), Paths[3].getTolerance(), Paths[3].getKc(), Paths[3].getMaxVelocity(), Paths[3].getTurnSpeed(), Paths[3].isFront());
        while (opModeIsActive() && (isRun || ElevateorBusy)) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() >= 10) {
                MyIntake.ShutDown();
                Output.setPosition(OutputClose);
            }
            if (MyPurePursuitGUI.findClosetPointIndex() == 39) {
                PlacingStoneTime = runtime.seconds();
            }
//                if (MyPurePursuitGUI.findClosetPointIndex() >= 40) {
//                    ElevateorBusy = PlacingStoneWhitTime(packet);
//                }
        }


        //        TODO Parking
        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[4].getWayPoints(), MyOdometry.getPosition(), Paths[4].getTolerance(), Paths[4].getKc(), Paths[4].getMaxVelocity(), Paths[4].getTurnSpeed(), Paths[4].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }
*/

//        TODO Telemetry
        while (!isStopRequested()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            packet.put("isRun", isRun.toString());
            dashboard.sendTelemetryPacket(packet);
        }

    }

    private void LocalUpdateGraphic() {/*
        packet.put("ElevateorBusy", ElevateorBusy);
        packet.put("isRun", isRun.toString());
        packet.put("LF", MyDriveTrain.LeftFront.getPower());
        packet.put("LB", MyDriveTrain.LeftBack.getPower());
        packet.put("RF", MyDriveTrain.RightFront.getPower());
        packet.put("RB", MyDriveTrain.RightBack.getPower());
        */
    }

    public boolean purePesuitRun() {
        double factor = 1;
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
            MyDriveTrain.arcade(factor*MyPurePursuitGUI.getYpower(), factor*petch*MyPurePursuitGUI.getXpower(), factor*MyPurePursuitGUI.getCpower());
        }
        MyPurePursuitGUI.updateGraghic(packet);
        LocalUpdateGraphic();
        dashboard.sendTelemetryPacket(packet);
        return true;
    }

}


