package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PP.PurePursuitGUI;
import org.firstinspires.ftc.teamcode.PP.OurPoint;

@Autonomous(name = "CheckList", group = "teamcode")
public class CheckList extends basicAutoCustumade {

    public PurePursuitGUI MyPurePursuitGUI;
    private OurPoint StartPosition = new OurPoint(-1.566, -0.8325, 270);
    public TelemetryPacket packet = null;
    private Boolean isRun = true;

    private static OurPoint[] TestPoints = {
            new OurPoint(-1, 1, 180),
            new OurPoint(-1.5, 1, 180)};
    private static double toleranceTest = 10;
    private static double KcTest = 4.5;
    private static double MaxVelocityTest = 1.5;
    private static double turnSpeedTest = 0.7;
    private static boolean frontTest = true;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        packet = new TelemetryPacket();
        BuildOdometry(StartPosition);

        isRun = true;
        MyPurePursuitGUI = new PurePursuitGUI(TestPoints, MyOdometry.getPosition(), toleranceTest, KcTest, MaxVelocityTest, turnSpeedTest, frontTest);
        while (opModeIsActive() && isRun) {
            isRun = purePesuitRun();
        }
        waitForStart();
        runtime.reset();

/*
        while (){
            MyElevator.ElevateWithEncoder(210,0.5,1);
        }
        while (){
            MyElevator.ElevateWithEncoder(0,0.3,1);
        }
  */
/*
        MyElevator.ElevateWithEncoder(-500, 0.3, 0.5);

        sleep(1000);
        Capstone.setPosition(CapstoneDown);
        Capstone.setPosition(CapstoneUp);
        sleep(1000);

        MyElevator.ElevateWithEncoder(0, 0.1, 0.003);

        MyIntake.maxIntake();
        sleep(1000);
        MyIntake.maxOuttake();

        MyDriveTrain.SetPower(0.8, 0.8, 0.8, 0.8);
        sleep(1000);
        MyDriveTrain.SetPower(0, 0, 0, 0);
        telemetry.addData("Odometry Y", MyOdometry.getPosition().getY());
        telemetry.update();
        sleep(4000);

        MyDriveTrain.SetPower(-0.8, 0.8, 0.8, -0.8);
        sleep(1000);
        MyDriveTrain.SetPower(0, 0, 0, 0);
        telemetry.addData("Odometry X", MyOdometry.getPosition().getX());
        telemetry.update();

        sleep(4000);

        Arm.setPosition(ArmOpen);
        Arm.setPosition(ArmClose);

        LeftServo.setPosition(LeftServoDown);
        RightServo.setPosition(RightServoDown);
        sleep(1000);
        LeftServo.setPosition(LeftServoUp);
        RightServo.setPosition(RightServoUp);

        sleep(1000);

        Output.setPosition(OutputClose);
        sleep(1000);
        Output.setPosition(OutputOpen);
        sleep(1000);

        ParkingMot.setPosition(ParkingMotIn);
        sleep(1000);
        ParkingMot.setPosition(ParkingMotOut);
        sleep(1000);

        telemetry.addData("down Magnet", downMagnetElevator.getState());
        telemetry.addData("CubeIn", cubeIn.getDistance(DistanceUnit.MM));
        telemetry.update();
        sleep(3000);

        telemetry.addData("left touch", LeftTouch.getState());
        telemetry.addData("right touch", RightTouch.getState());
        telemetry.update();
        sleep(2000);
*/

//        TODO Telemetry
        while (!isStopRequested()) {
            packet = new TelemetryPacket();
            MyPurePursuitGUI.updateGraghic(packet);
            packet.put("isRun", isRun.toString());
        }

    }

    private void LocalUpdateGraphic() {
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


