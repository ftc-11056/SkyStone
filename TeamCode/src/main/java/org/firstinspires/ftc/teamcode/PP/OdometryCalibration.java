package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.PP.OurPoint;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

@TeleOp(name="OdometryCalibration", group="Iterative Opmode")
@Disabled
public class OdometryCalibration extends Robot {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public ArrayList<Double> meterForAng = new ArrayList<>();
    public ArrayList<Double> angle = new ArrayList<>();
    ArrayList<OurPoint> RL = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        MyOdometry = new Odometry(new OurPoint(0,-1.61));
        // run until the end of the match (driver presses STOP)
        angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        FtcDashboard dashboard = FtcDashboard.getInstance();
//        File OdometryCalibrate = AppUtil.getInstance().getSettingsFile("OdometryCalibrate.txt");
        waitForStart();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("dAngle", MyOdometry.dAngle);
        packet.put("meterForAng" , meterForAng);
        packet.put("dRight" , MyOdometry.dRight);
        packet.put("dLeft" , MyOdometry.dLeft);
        dashboard.sendTelemetryPacket(packet);
        sleep(2000);
        while(opModeIsActive() && angles.firstAngle < Math.toRadians(90)){
            packet = new TelemetryPacket();
            double power = 1;
//            if(angles.firstAngle > Math.toRadians(60))
//                power = 0.25;
            MyDriveTrain.LeftFront.setPower(-power);
            MyDriveTrain.LeftBack.setPower(-power);
            MyDriveTrain.RightFront.setPower(power);
            MyDriveTrain.RightBack.setPower(power);
            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            double currentTime = runtime.seconds();
            double odometryRight = -(MyDriveTrain.RightFront.getCurrentPosition());
            double odometryLeft = -(MyDriveTrain.LeftFront.getCurrentPosition());
            double odometryHorizental = -(MyDriveTrain.RightBack.getCurrentPosition());
            MyOdometry.setAll(odometryRight, odometryLeft, odometryHorizental, currentTime);
            packet.put("dRight" , MyOdometry.dRight);
            packet.put("dLeft" , MyOdometry.dLeft);
            RL.add(new OurPoint(MyOdometry.dRight,MyOdometry.dLeft));
            double dAngle = MyOdometry.dAngle;
            angle.add((dAngle));
            double meterForAng = MyOdometry.currentAngle / dAngle;
            packet.put("meterForAng" , meterForAng);
            this.meterForAng.add(meterForAng);
            packet.put("dAngle", MyOdometry.dAngle);
            dashboard.sendTelemetryPacket(packet);
        }
        MyDriveTrain.stop();
        double meterForAngle = 0;
        for(Double d: meterForAng){
            packet.addLine(String.valueOf(d));
            meterForAngle += d;
        }
        meterForAngle /= meterForAng.size();
        packet.addLine("Direction" + String.valueOf(MyOdometry.getDirection()));
        packet.addLine("dSide" + String.valueOf(MyOdometry.dSide));
        packet.addLine("TicForAngle1 "+String.valueOf(meterForAngle));
        packet.addLine("TicForAngle2 "+String.valueOf(MyOdometry.dSide / MyOdometry.getDirection()));
        dashboard.sendTelemetryPacket(packet);
//          ReadWriteFile.writeFile(OdometryCalibrate,String.valueOf(ticForAngle));
//        while (opModeIsActive()) {
//            packet = new TelemetryPacket();
//            packet.put( "tics",-(MyDriveTrain.RightBack.getCurrentPosition()));
//            packet.put("ticForAngle", ticForAngle);
//            dashboard.sendTelemetryPacket(packet);
//        }
    }

}