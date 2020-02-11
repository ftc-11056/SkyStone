package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCustomade;
import org.firstinspires.ftc.teamcode.basicAutoCustumade;


@Autonomous(name = "Autonom_Red_3Cubes", group = "teamcode")
public class Autonom_Red_3Cubes extends basicAutoCustumade {

    public PurePursuitGUI MyPurePursuitGUI;
    private OurPoint StartPosition = new OurPoint(1.566, -0.8325, 90);
    private Boolean isRun = true;
    public double deltaFromFlatAngle = 0;
    private boolean ElevateorBusy = true;
    private double petchX = 1;
    private double pechY = 1;
    private double distanceToCenter;
    private double factor = 1;
    private double Cpower = 1;
    private boolean IMUTurn = false;
    Path[] Paths = Paths_Library_Red_3Cubes.LeftPaths;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        BuildOdometry(StartPosition);
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//        Mikum = skystoneDetector.getScreenPosition().y;

        int numOfCheck = 1;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[numOfCheck].getWayPoints(), MyOdometry.getPosition(), Paths[numOfCheck].getTolerance(), Paths[numOfCheck].getKc(), Paths[numOfCheck].getMaxVelocity(), Paths[numOfCheck].getTurnSpeed(), Paths[numOfCheck].isFront());
        while (!isStarted()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            dashboard.sendTelemetryPacket(packet);
//            Mikum = skystoneDetector.getScreenPosition().y;
//            telemetry.addData("skystone", Mikum);
//            telemetry.update();
        }

       /* Mikum = 150;
        if(Mikum <= 90){
            Paths = Paths_Library_Red_3Cubes.LeftPaths;
        }
        else if (Mikum >= 210){
            Paths = Paths_Library_Red_3Cubes.RightPaths;
        }
        else {
            Paths = Paths_Library_Red_3Cubes.CenterPaths;
        }
*/
        runtime.reset();
//        webcam.closeCameraDevice();
        waitForStart();

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

//        MyDriveTrain.Verification(cubeIn,cubeNotInMM,packet,dashboard);

        factor = 1;
        isRun = true;
        boolean isTouch = false;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[1].getWayPoints(), MyOdometry.getPosition(), Paths[1].getTolerance(), Paths[1].getKc(), Paths[1].getMaxVelocity(), Paths[1].getTurnSpeed(), Paths[1].isFront());
        while (opModeIsActive() && isRun && !isTouch) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() >= 20) {
                MyIntake.ShutDown();
                MyPurePursuitGUI.setTurnSpeed(1);
                Output.setPosition(OutputClose);
            }

            if (MyPurePursuitGUI.findClosetPointIndex() >= 33) {
                LeftServo.setPosition(LeftServoMiddle);
                RightServo.setPosition(RightServoMiddle);
                MyPurePursuitGUI.setKv(0.4);
                if(!LeftTouch.getState() || !RightTouch.getState()){
                    isTouch = true;

                }
            }
            if(MyPurePursuitGUI.findClosetPointIndex() >= 46){
                IMUTurn = true;
                Cpower = IMUError(180,0.6);
            }
        }
        MyOdometry.getPosition().setDegAngle(180 + MyDriveTrain.getAngle());
        IMUTurn = false;

        TouchFoundation(LeftTouch,RightTouch);

        Arm.setPosition(ArmOpen);
        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(400);

        Paths[2].getWayPoints()[0].setX(MyOdometry.getPosition().getX());

        isRun = true;
        PlacingStoneTime = runtime.seconds();
        MyPurePursuitGUI = new PurePursuitGUI(Paths[2].getWayPoints(), MyOdometry.getPosition(), Paths[2].getTolerance(), Paths[2].getKc(), Paths[2].getMaxVelocity(), Paths[2].getTurnSpeed(), Paths[2].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
            PlacingStoneWhitTime();
        }

        Arm.setPosition(ArmClose);
        RotateP1(90,0.8,10,0.05);
        MyOdometry.getPosition().setDegAngle(180 + MyDriveTrain.getAngle());
        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);
        sleep(400);

        Paths[3].getWayPoints()[0] = new OurPoint(MyOdometry.getPosition());
        Paths[3].getWayPoints()[0].setDegAngle(180);

        distanceToCenter = 1;
        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[3].getWayPoints(), MyOdometry.getPosition(), Paths[3].getTolerance(), Paths[3].getKc(), Paths[3].getMaxVelocity(), Paths[3].getTurnSpeed(), Paths[3].isFront());
        while (opModeIsActive() && isRun && distanceToCenter >= 0.14) {
            isRun = purePesuitRun();
            distanceToCenter = MyMath.distance(MyOdometry.getPosition(), Paths[3].getWayPoints()[Paths[3].getWayPoints().length-1]);
            if (MyPurePursuitGUI.findClosetPointIndex() >= 48) {
                Output.setPosition(OutputOpen);
                MyIntake.maxIntake();
                factor = 0.3;
            }
        }
        factor = 1;
//        MyDriveTrain.Verification(cubeIn,cubeNotInMM,packet,dashboard);

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[4].getWayPoints(), MyOdometry.getPosition(), Paths[4].getTolerance(), Paths[4].getKc(), Paths[4].getMaxVelocity(), Paths[4].getTurnSpeed(), Paths[4].isFront());
        while (opModeIsActive() && (isRun)) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() >= 15) {
                MyIntake.ShutDown();
                Output.setPosition(OutputClose);
            }
            if(MyPurePursuitGUI.findClosetPointIndex() == 37){
                PlacingStoneTime = runtime.seconds();
                Arm.setPosition(ArmOpen);
            }
            if(MyPurePursuitGUI.findClosetPointIndex() >= 38){
                ElevateorBusy = PlacingStoneWhitTime();
            }
        }

        isRun = true;
        boolean isCubeIn = false;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[5].getWayPoints(), MyOdometry.getPosition(), Paths[5].getTolerance(), Paths[5].getKc(), Paths[5].getMaxVelocity(), Paths[5].getTurnSpeed(), Paths[5].isFront());
        while (opModeIsActive() && isRun && !isCubeIn) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() <= 7) {
                MyIntake.ShutDown();
                Output.setPosition(OutputOpen);
                Arm.setPosition(ArmClose);
            }
            if (MyPurePursuitGUI.findClosetPointIndex() >= 15) {
                MyIntake.maxIntake();
            }
            if(cubeIn.getDistance(DistanceUnit.MM) < cubeNotInMM){
                 isCubeIn = true;
                 MyIntake.ShutDown();
            }

        }


        Paths[5].getWayPoints()[0] = new OurPoint(MyOdometry.getPosition());
        Paths[5].getWayPoints()[0].setDegAngle(140);

/*
        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[6].getWayPoints(), MyOdometry.getPosition(), Paths[6].getTolerance(), Paths[6].getKc(), Paths[6].getMaxVelocity(), Paths[6].getTurnSpeed(), Paths[6].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
            if (MyPurePursuitGUI.findClosetPointIndex() <= 3) {
                Output.setPosition(OutputClose);
            }
            if (MyPurePursuitGUI.findClosetPointIndex() == 30) {
                PlacingStoneTime = runtime.seconds();
                Arm.setPosition(ArmClose);

            }
            if(MyPurePursuitGUI.findClosetPointIndex() >= 31){
                ElevateorBusy = PlacingStoneWhitTime(packet);
            }
        }



        //        TODO Parking
        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(Paths[6].getWayPoints(), MyOdometry.getPosition(), Paths[6].getTolerance(), Paths[6].getKc(), Paths[6].getMaxVelocity(), Paths[6].getTurnSpeed(), Paths[6].isFront());
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
            if(MyPurePursuitGUI.findClosetPointIndex() <= 3){
                Arm.setPosition(ArmClose);
            }
        }
*/

//        TODO Telemetry
        while (opModeIsActive()&&!isStopRequested()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            packet.put("isRun", isRun.toString());
            dashboard.sendTelemetryPacket(packet);
        }

    }

    private void LocalUpdateGraphic() {
        packet.put("IMU angle", MyDriveTrain.getAngle());
        packet.put("deltaFromFlatAngle", Math.toDegrees(deltaFromFlatAngle));
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


