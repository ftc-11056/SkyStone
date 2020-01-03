package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name = "Autonom_2_cube_goBILDA_Robot", group = "teamcode")
public class Autonom_2_cube_goBILDA_Robot extends Robot {


    public PurePursuitGUI MyPurePursuitGUI;
    public FtcDashboard dashboard;
    private OurPoint StartPosition = new OurPoint(1.566, -0.875, 90);
    private OurPoint[] Left1 = {
            new OurPoint(1.566, -0.875, 130),
            new OurPoint(0.8, -0.7, 130),
            new OurPoint(0.51,-1,130)};
    private double toleranceLeft1 = 50;
    private double KcLeft1 = 0.5;
    private double MaxVelocityLeft1 = 0.5;
    private double turnSpeedLeft1 = 0.5;
    private boolean frontLeft1 = true;

    private OurPoint[] LeftFoundation = {
            new OurPoint(0.5,-1.02,180),
            new OurPoint(0.9, -0.35, 180),
            new OurPoint(0.90, 0.55, 270),
            new OurPoint(0.74, 1.15, 270)};
    private double toleranceLeftFoundation = 40;
    private double KcLeftFoundation = 3;
    private double MaxVelocityLeftFoundation = 1.5;
    private double turnSpeedLeftFoundation = 1.2;
    private boolean frontLeftFoundation = false;

    private OurPoint[] Left2 = {
            new OurPoint(0.74, 1.15, 270),
            new OurPoint(1.2, 1.15,180),
            new OurPoint(1.2, 0.40, 180),
            new OurPoint(1.2, -0.85, 150),
            new OurPoint(0.65, -1.4, 150),
            new OurPoint(0.35, -1.45, 150)};
    private double toleranceLeft2 = 60;
    private double KcLeft2 = 2.5;
    private double MaxVelocityLeft2 = 1.5;
    private double turnSpeedLeft2 = 1.5;
    private boolean frontLeft2 = true;

    private OurPoint[] LeftFoundation2 = {
            new OurPoint(0.5, -1.46, 180),
            new OurPoint(0.9, -0.85, 180),
            new OurPoint(0.9, 0.15, 180),
            //new OurPoint(1.2, 0.5, 180),
            new OurPoint(0.9, 1.1, 180)};
    private double toleranceLeftFoundation2 = 61.9;
    private double KcLeftFoundation2 = 4.5;
    private double MaxVelocityLeftFoundation2 = 1.5;
    private double turnSpeedLeftFoundation2 = 0.7;
    private boolean frontLeftFoundation2 = false;

    private OurPoint[] Parking = {
            new OurPoint(1.2, 1.1, 180),
            new OurPoint(0.9, 0.40, 180)};
    private double toleranceParking = 10;
    private double KcParking = 4.5;
    private double MaxVelocityParking = 1.5;
    private double turnSpeedParking = 0.7;
    private boolean frontParking = true;
    public TelemetryPacket packet = null;
    private Boolean isRun = true;
    public double deltaFromFlatAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        BuildOdometry(StartPosition);
        MyPurePursuitGUI = new PurePursuitGUI(Left1, MyOdometry.getPosition(), toleranceLeft1, KcLeft1, MaxVelocityLeft1, turnSpeedLeft1, frontLeft1);
        while (!isStarted()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            dashboard.sendTelemetryPacket(packet);
        }
        runtime.reset();
        waitForStart();


        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Left1, MyOdometry.getPosition(), toleranceLeft1, KcLeft1, MaxVelocityLeft1, turnSpeedLeft1, frontLeft1);
        MyPurePursuitGUI.setKv(1);
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() >= 13) {
                MyPurePursuitGUI.setKv(0.6);
                MyIntake.maxIntake();
            }
        }


        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(LeftFoundation, MyOdometry.getPosition(), toleranceLeftFoundation, KcLeftFoundation, MaxVelocityLeftFoundation, turnSpeedLeftFoundation, frontLeftFoundation);
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() >= 20) {
                MyIntake.ShutDown();
            }

            if (MyPurePursuitGUI.findClosetPointIndex() >= 42) {
                LeftServo.setPosition(0.85);
                RightServo.setPosition(0.65);
            }

        }

        Output.setPosition(OutputDown);
        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(400);

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Left2, MyOdometry.getPosition(), toleranceLeft2, KcLeft2, MaxVelocityLeft2, turnSpeedLeft2, frontLeft2);
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();

            setPointIndexStartElavator(0);
            if (MyPurePursuitGUI.findClosetPointIndex() >= 0 && MyPurePursuitGUI.findClosetPointIndex() <= 35) {
                PlacingStone(MyPurePursuitGUI.findClosetPointIndex(),packet);
            }

            deltaFromFlatAngle = Math.abs(MyOdometry.getDirection() - Math.toRadians(270));
            if (MyPurePursuitGUI.findClosetPointIndex() >= 20) {
                LeftServo.setPosition(LeftServoUp);
                RightServo.setPosition(RightServoUp);
            }

            if(MyPurePursuitGUI.findClosetPointIndex() >= 20){
                MyPurePursuitGUI.setKv(0.6);
                MyIntake.maxIntake();
            }
        }



        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(LeftFoundation2, MyOdometry.getPosition(), toleranceLeftFoundation2, KcLeftFoundation2,MaxVelocityLeftFoundation2 ,turnSpeedLeftFoundation2 ,frontLeftFoundation2);
        while(opModeIsActive() && isRun){
            isRun =  purePesuitRun();
            if(MyPurePursuitGUI.findClosetPointIndex() >= 10){
                MyIntake.ShutDown();
            }
        }
/*
        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Parking, MyOdometry.getPosition(), toleranceParking, KcParking, MaxVelocityParking, turnSpeedParking, frontParking);
        while(opModeIsActive() && isRun){
            isRun =  purePesuitRun();
        }
*/
        while (!isStopRequested()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            packet.put("isRun", isRun.toString());
            dashboard.sendTelemetryPacket(packet);
        }

    }

    public boolean purePesuitRun(){
        packet = new TelemetryPacket();
        double currentTime = runtime.seconds();
        updateOdometry();
        MyPurePursuitGUI.UpdatePowerByRobotPosition(packet,currentTime, MyOdometry.getPosition(), MyOdometry.getVelocityX(), MyOdometry.getVelocityY());
        if(MyPurePursuitGUI.stop){
            MyDriveTrain.stop();
            packet.addLine("the end");
            dashboard.sendTelemetryPacket(packet);
            return false;
        }
        else {
            MyDriveTrain.arcade(MyPurePursuitGUI.getYpower(), MyPurePursuitGUI.getXpower(), MyPurePursuitGUI.getCpower());
        }
        MyPurePursuitGUI.updateGraghic(packet);
        packet.put("isRun",isRun.toString());
        dashboard.sendTelemetryPacket(packet);
        return true;
    }
}


