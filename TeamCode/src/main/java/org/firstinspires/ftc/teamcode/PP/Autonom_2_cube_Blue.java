package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name = "Autonom_2_cube_Blue", group = "teamcode")
public class Autonom_2_cube_Blue extends Robot {

    public String CubePlace = "Center";
    public PurePursuitGUI MyPurePursuitGUI;
    public FtcDashboard dashboard;
    private OurPoint StartPosition = new OurPoint(-1.566, -0.875, 270);
    //    TODO Left Points
    private OurPoint[] Left1 = {
            new OurPoint(-1.566, -0.875, 230),
            new OurPoint(-0.85, -0.6, 230),
            new OurPoint(-0.55, -1, 230)};
    private double toleranceLeft1 = 53;
    private double KcLeft1 = 0.5;
    private double MaxVelocityLeft1 = 0.4;
    private double turnSpeedLeft1 = 0.4;
    private boolean frontLeft1 = true;

    private OurPoint[] LeftFoundation = {
            new OurPoint(-0.55, -1, 180),
            new OurPoint(-0.9, -0.35, 180),
            new OurPoint(-0.9, 0.55, 90),
            new OurPoint(-0.53, 1.21, 90)};
    private double toleranceLeftFoundation = 75;
    private double KcLeftFoundation = 3;
    private double MaxVelocityLeftFoundation = 1.5;
    private double turnSpeedLeftFoundation = 1.2;
    private boolean frontLeftFoundation = false;

    private OurPoint[] Left2 = {
            new OurPoint(-0.53, 1.21, 90),
            new OurPoint(-1.55, 1.23, 180),
            new OurPoint(-0.85, 0.30, 180),
            new OurPoint(-0.85, -1.25, 180),
            new OurPoint(-0.4, -1.3, 180),
            new OurPoint(-0.4, -1.43, 180)};
    private double toleranceLeft2 = 120;
    private double KcLeft2 = 1.5;
    private double MaxVelocityLeft2 = 1;
    private double turnSpeedLeft2 = 1.5;
    private boolean frontLeft2 = true;


    private OurPoint[] LeftFoundation2 = {
            new OurPoint(-0.52, -1.43, 180),
            new OurPoint(-0.9, -0.85, 180),
            new OurPoint(-0.9, 0.15, 180),
            new OurPoint(-1.2, 0.5, 180),
            new OurPoint(-1.2, 1.05, 180)};
    private double toleranceLeftFoundation2 = 55;
    private double KcLeftFoundation2 = 2.5;
    private double MaxVelocityLeftFoundation2 = 1.5;
    private double turnSpeedLeftFoundation2 = 0.2;
    private boolean frontLeftFoundation2 = false;

    //    TODO Center Points
    private OurPoint[] Center1 = {
            new OurPoint(-1.566, -0.875, 230),
            new OurPoint(-0.8,-0.4,230),
            new OurPoint(-0.55, -0.7, 230)};
    private double toleranceCenter1 = 53;
    private double KcCenter1 = 0.5;
    private double MaxVelocityCenter1 = 0.3;
    private double turnSpeedCenter1 = 0.3;
    private boolean frontCenter1 = true;

    private OurPoint[] CenterFoundation = {
            new OurPoint(-0.55, -0.7, 180),
            new OurPoint(-0.9, -0.35, 180),
            new OurPoint(-0.9, 0.55, 90),
            new OurPoint(-0.53, 1.21, 90)};
    private double toleranceCenterFoundation = 45;
    private double KcCenterFoundation = 3;
    private double MaxVelocityCenterFoundation = 1.5;
    private double turnSpeedCenterFoundation = 1.2;
    private boolean frontCenterFoundation = false;

    private OurPoint[] Center2 = {
            new OurPoint(-0.53, 1.21, 90),
            new OurPoint(-1.55, 1.23, 180),
            new OurPoint(-0.85, 0.30, 180),
            new OurPoint(-0.85, -1, 180),
            new OurPoint(-0.3, -1, 180),
            new OurPoint(-0.3, -1.2, 180)};
    private double toleranceCenter2 = 120;
    private double KcCenter2 = 1.5;
    private double MaxVelocityCenter2 = 1;
    private double turnSpeedCenter2 = 1.5;
    private boolean frontCenter2 = true;

    private OurPoint[] CenterFoundation2 = {
            new OurPoint(-0.52, -1.43, 180),
            new OurPoint(-0.9, -0.7, 180),
            new OurPoint(-0.9, 0.15, 180),
            new OurPoint(-1.2, 0.5, 180),
            new OurPoint(-1.2, 1.05, 180)};
    private double toleranceCenterFoundation2 = 55;
    private double KcCenterFoundation2 = 2.5;
    private double MaxVelocityCenterFoundation2 = 1.5;
    private double turnSpeedCenterFoundation2 = 1.5;
    private boolean frontCenterFoundation2 = false;

    //    TODO Right Points
    private OurPoint[] Right1 = {
            new OurPoint(-1.566, -0.875, 230),
            new OurPoint(-0.8, -0.2, 230),
            new OurPoint(-0.55, -0.5, 230)};
    private double toleranceRight1 = 53;
    private double KcRight1 = 0.5;
    private double MaxVelocityRight1 = 0.3;
    private double turnSpeedRight1 = 0.3;
    private boolean frontRight1 = true;

    private OurPoint[] RightFoundation = {
            new OurPoint(-0.55, -0.5, 180),
            new OurPoint(-0.9, -0.35, 180),
            new OurPoint(-0.9, 0.55, 90),
            new OurPoint(-0.53, 1.21, 90)};
    private double toleranceRightFoundation = 65;
    private double KcRightFoundation = 3;
    private double MaxVelocityRightFoundation = 1.5;
    private double turnSpeedRightFoundation = 1.2;
    private boolean frontRightFoundation = false;

    private OurPoint[] Right2 = {
            new OurPoint(-0.53, 1.21, 90),
            new OurPoint(-1.55, 1.18, 180),
            new OurPoint(-0.8, 0.3, 180),
            new OurPoint(-0.8, -0.8, 230),
            new OurPoint(-0.55, -0.9, 230),
            new OurPoint(-0.45,-0.95, 230)};
    private double toleranceRight2 = 120;
    private double KcRight2 = 1.5;
    private double MaxVelocityRight2 = 1;
    private double turnSpeedRight2 = 1.5;
    private boolean frontRight2 = true;

    private OurPoint[] RightFoundation2 = {
            new OurPoint(-0.45, -0.9, 180),
            new OurPoint(-0.85, -0.6, 180),
            new OurPoint(-0.85, 0.15, 180),
            new OurPoint(-1.2, 0.5,180),
            new OurPoint(-1.2, 1.05, 180)};
    private double toleranceRightFoundation2 = 53;
    private double KcRightFoundation2 = 2.5;
    private double MaxVelocityRightFoundation2 = 1.5;
    private double turnSpeedRightFoundation2 = 0.2;
    private boolean frontRightFoundation2 = false;

    //    TODO Parking Points
    private OurPoint[] Parking = {
            new OurPoint(-1.2, 0.97, 180),
            new OurPoint(-0.8, 0, 180)};
    private double toleranceParking = 10;
    private double KcParking = 4.5;
    private double MaxVelocityParking = 1.5;
    private double turnSpeedParking = 0.7;
    private boolean frontParking = true;
    public TelemetryPacket packet = null;
    private Boolean isRun = true;
    public double deltaFromFlatAngle = 0;
    private boolean ElevateorBusy = true;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        BuildOdometry(StartPosition);
        MyPurePursuitGUI = new PurePursuitGUI(Center2, MyOdometry.getPosition(), toleranceCenter2, KcCenter2, MaxVelocityCenter2, turnSpeedCenter2, frontCenter2);
        while (!isStarted()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            dashboard.sendTelemetryPacket(packet);
        }
        runtime.reset();
        waitForStart();

//        TODO Left Lane
        if (CubePlace == "Left") {
            isRun = true;
            MyPurePursuitGUI = new PurePursuitGUI(Left1, MyOdometry.getPosition(), toleranceLeft1, KcLeft1, MaxVelocityLeft1, turnSpeedLeft1, frontLeft1);
            MyPurePursuitGUI.setKv(1);
            while (opModeIsActive() && isRun) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() >= 16) {
                    MyPurePursuitGUI.setKv(0.65);
                    MyIntake.maxIntake();
                }
            }


            isRun = true;
            MyPurePursuitGUI = new PurePursuitGUI(LeftFoundation, MyOdometry.getPosition(), toleranceLeftFoundation, KcLeftFoundation, MaxVelocityLeftFoundation, turnSpeedLeftFoundation, frontLeftFoundation);
            while (opModeIsActive() && isRun) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() == 1) {
                    MyIntake.ShutDown();
                }

                if (MyPurePursuitGUI.findClosetPointIndex() >= 42) {
                    LeftServo.setPosition(0.85);
                    RightServo.setPosition(0.85);
                }

            }

            MyIntake.maxOuttake();
            sleep(1000);
            MyIntake.ShutDown();
            Output.setPosition(OutputDown);
//            LeftServo.setPosition(LeftServoDown);
//            RightServo.setPosition(RightServoDown);
            sleep(400);


            isRun = true;
            MyPurePursuitGUI = new PurePursuitGUI(Left2, MyOdometry.getPosition(), toleranceLeft2, KcLeft2, MaxVelocityLeft2, turnSpeedLeft2, frontLeft2);
            while (opModeIsActive() && isRun) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() <= 12) {
                    MyPurePursuitGUI.setTurnSpeed(0.2);
                } else {
                    packet.addLine("switch");
                    MyPurePursuitGUI.setTurnSpeed(2.5);
                }

                setPointIndexStartElavator(0);
//                if (MyPurePursuitGUI.findClosetPointIndex() >= 0 && MyPurePursuitGUI.findClosetPointIndex() <= 45) {
//                    PlacingStoneWhitPoints(MyPurePursuitGUI.findClosetPointIndex(), packet);
//                }

                deltaFromFlatAngle = Math.abs(MyOdometry.getDirection() - Math.toRadians(270));
                if (MyPurePursuitGUI.findClosetPointIndex() >= 35) {
                    LeftServo.setPosition(LeftServoUp);
                    RightServo.setPosition(RightServoUp);
                    MyPurePursuitGUI.setKv(0.5);
                    MyPurePursuitGUI.setTurnSpeed(0.7);
                }

                if (MyPurePursuitGUI.findClosetPointIndex() >= 50) {
                    Output.setPosition(OutputUp);
                    MyIntake.maxIntake();
                }
            }


            isRun = true;
            ElevateorBusy = true;
            MyPurePursuitGUI = new PurePursuitGUI(LeftFoundation2, MyOdometry.getPosition(), toleranceLeftFoundation2, KcLeftFoundation2, MaxVelocityLeftFoundation2, turnSpeedLeftFoundation2, frontLeftFoundation2);
            while (opModeIsActive() && (isRun || ElevateorBusy)) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() >= 10) {
                    MyIntake.ShutDown();
                    Output.setPosition(OutputDown);
                }
                if(MyPurePursuitGUI.findClosetPointIndex() == 39){
                    PlacingStoneTime = runtime.seconds();
                }
//                if (MyPurePursuitGUI.findClosetPointIndex() >= 40) {
//                    ElevateorBusy = PlacingStoneWhitTime(packet);
//                }
            }
        }

        //        TODO Center Lane
        else if (CubePlace == "Center") {
            isRun = true;
            MyPurePursuitGUI = new PurePursuitGUI(Center1, MyOdometry.getPosition(), toleranceCenter1, KcCenter1, MaxVelocityCenter1, turnSpeedCenter1, frontCenter1);
            MyPurePursuitGUI.setKv(1);
            while (opModeIsActive() && isRun) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() >= 16) {
                    MyPurePursuitGUI.setKv(0.7);
                    MyIntake.maxIntake();
                }
            }


            isRun = true;
            MyPurePursuitGUI = new PurePursuitGUI(CenterFoundation, MyOdometry.getPosition(), toleranceCenterFoundation, KcCenterFoundation, MaxVelocityCenterFoundation, turnSpeedCenterFoundation, frontCenterFoundation);
            while (opModeIsActive() && isRun) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() == 1) {
                    MyIntake.ShutDown();
                }

                if (MyPurePursuitGUI.findClosetPointIndex() >= 42) {
                    LeftServo.setPosition(0.85);
                    RightServo.setPosition(0.85);
                }

            }

            MyIntake.maxOuttake();
            sleep(1000);
            MyIntake.ShutDown();
            Output.setPosition(OutputDown);
//            LeftServo.setPosition(LeftServoDown);
//            RightServo.setPosition(RightServoDown);
            sleep(400);

            isRun = true;
            MyPurePursuitGUI = new PurePursuitGUI(Center2, MyOdometry.getPosition(), toleranceCenter2, KcCenter2, MaxVelocityCenter2, turnSpeedCenter2, frontCenter2);
            while (opModeIsActive() && isRun) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() <= 12) {
                    MyPurePursuitGUI.setTurnSpeed(0.2);
                } else {
                    packet.addLine("switch");
                    MyPurePursuitGUI.setTurnSpeed(2.5);
                }

                setPointIndexStartElavator(0);
//            if (MyPurePursuitGUI.findClosetPointIndex() >= 0 && MyPurePursuitGUI.findClosetPointIndex() <= 45) {
//                PlacingStoneWhitPoints(MyPurePursuitGUI.findClosetPointIndex(),packet);
//            }

                deltaFromFlatAngle = Math.abs(MyOdometry.getDirection() - Math.toRadians(270));
                if (MyPurePursuitGUI.findClosetPointIndex() >= 35) {
                    LeftServo.setPosition(LeftServoUp);
                    RightServo.setPosition(RightServoUp);
                    MyPurePursuitGUI.setKv(0.5);
                    MyPurePursuitGUI.setTurnSpeed(0.65);
                }

                if (MyPurePursuitGUI.findClosetPointIndex() >= 50) {
                    Output.setPosition(OutputUp);
                    MyIntake.maxIntake();
                }
            }


            isRun = true;
            ElevateorBusy = true;
            MyPurePursuitGUI = new PurePursuitGUI(CenterFoundation2, MyOdometry.getPosition(), toleranceCenterFoundation2, KcCenterFoundation2, MaxVelocityCenterFoundation2, turnSpeedCenterFoundation2, frontCenterFoundation2);
            while (opModeIsActive() && isRun || ElevateorBusy) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() >= 10) {
                    MyIntake.ShutDown();
                    Output.setPosition(OutputDown);
                }
                if (MyPurePursuitGUI.findClosetPointIndex() == 39){
                    PlacingStoneTime = runtime.seconds();
                }
//                if(MyPurePursuitGUI.findClosetPointIndex() >= 40){
//                    ElevateorBusy = PlacingStoneWhitTime(packet);
//                }
            }
        }

//        gggggg

        //        TODO Right Lane
        else if (CubePlace == "Right") {
            isRun = true;
            MyPurePursuitGUI = new PurePursuitGUI(Right1, MyOdometry.getPosition(), toleranceRight1, KcRight1, MaxVelocityRight1, turnSpeedRight1, frontRight1);
            MyPurePursuitGUI.setKv(1);
            while (opModeIsActive() && isRun) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() >= 16) {
                    MyPurePursuitGUI.setKv(0.8);
                    MyIntake.maxIntake();
                }
            }


            isRun = true;
            MyPurePursuitGUI = new PurePursuitGUI(RightFoundation, MyOdometry.getPosition(), toleranceRightFoundation, KcRightFoundation, MaxVelocityRightFoundation, turnSpeedRightFoundation, frontRightFoundation);
            while (opModeIsActive() && isRun) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() == 1) {
                    MyIntake.ShutDown();
                }

                if (MyPurePursuitGUI.findClosetPointIndex() >= 42) {
                    LeftServo.setPosition(0.85);
                    RightServo.setPosition(0.85);
                }

            }

            sleep(4500);
            Output.setPosition(OutputDown);
            LeftServo.setPosition(LeftServoDown);
            RightServo.setPosition(RightServoDown);
            sleep(400);

            isRun = true;
            MyPurePursuitGUI = new PurePursuitGUI(Right2, MyOdometry.getPosition(), toleranceRight2, KcRight2, MaxVelocityRight2, turnSpeedRight2, frontRight2);
            while (opModeIsActive() && isRun) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() <= 12) {
                    MyPurePursuitGUI.setTurnSpeed(0.2);
                } else {
                    packet.addLine("switch");
                    MyPurePursuitGUI.setTurnSpeed(2.5);
                }

                setPointIndexStartElavator(0);
//            if (MyPurePursuitGUI.findClosetPointIndex() >= 0 && MyPurePursuitGUI.findClosetPointIndex() <= 45) {
//                PlacingStoneWhitPoints(MyPurePursuitGUI.findClosetPointIndex(),packet);
//            }

                deltaFromFlatAngle = Math.abs(MyOdometry.getDirection() - Math.toRadians(270));
                if (MyPurePursuitGUI.findClosetPointIndex() >= 35) {
                    LeftServo.setPosition(LeftServoUp);
                    RightServo.setPosition(RightServoUp);
                    MyPurePursuitGUI.setKv(0.7);
                    MyPurePursuitGUI.setTurnSpeed(0.65);
                }

                if (MyPurePursuitGUI.findClosetPointIndex() >= 50) {
                    Output.setPosition(OutputUp);
                    MyIntake.maxIntake();
                }
            }

            isRun = true;
            MyPurePursuitGUI = new PurePursuitGUI(RightFoundation2, MyOdometry.getPosition(), toleranceRightFoundation2, KcRightFoundation2, MaxVelocityRightFoundation2, turnSpeedRightFoundation2, frontRightFoundation2);
            while (opModeIsActive() && isRun) {
                isRun = purePesuitRun();
                if (MyPurePursuitGUI.findClosetPointIndex() >= 10) {
                    MyIntake.ShutDown();
                    Output.setPosition(OutputDown);
                }
                if (MyPurePursuitGUI.findClosetPointIndex() == 39){
                    PlacingStoneTime = runtime.seconds();
                }
//                if(MyPurePursuitGUI.findClosetPointIndex() >= 40){
//                    ElevateorBusy = PlacingStoneWhitTime(packet);
//                }

            }


        }

        //        TODO Parking
        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Parking, MyOdometry.getPosition(), toleranceParking, KcParking, MaxVelocityParking, turnSpeedParking, frontParking);
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

    private void LocalUpdateGraphic(){
        packet.put("ElevateorBusy",ElevateorBusy);
        packet.put("isRun", isRun.toString());
        packet.put("LF", MyDriveTrain.LeftFront.getPower());
        packet.put("LB", MyDriveTrain.LeftBack.getPower());
        packet.put("RF", MyDriveTrain.RightFront.getPower());
        packet.put("RB", MyDriveTrain.RightBack.getPower());
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
            MyDriveTrain.arcade(MyPurePursuitGUI.getYpower(), MyPurePursuitGUI.getXpower(), MyPurePursuitGUI.getCpower());
        }
        MyPurePursuitGUI.updateGraghic(packet);
        LocalUpdateGraphic();
        dashboard.sendTelemetryPacket(packet);
        return true;
    }

}


