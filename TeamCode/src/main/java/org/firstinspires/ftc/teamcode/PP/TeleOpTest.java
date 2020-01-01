package org.firstinspires.ftc.teamcode.PP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

@TeleOp(name = "TeleOpTest", group = "teamcode")
public class TeleOpTest extends Robot {


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        double odometryRight = 0;
        double odometryLeft = 0;
        double odometryHorizental = 0;
        BuildOdometry(new OurPoint(1.566, -0.875, 90));
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        waitForStart();
        runtime.reset();
        double start,currentTime = runtime.seconds();
        start = currentTime;
        /*while(opModeIsActive() && (currentTime - start) < 0.8){
            packet = new TelemetryPacket();
            MyDriveTrain.arcade(1,0,0);
            currentTime = runtime.seconds();
            odometryRight = -(MyDriveTrain.RightFront.getCurrentPosition());
            odometryLeft = -(MyDriveTrain.LeftFront.getCurrentPosition());
            odometryHorizental = -(MyDriveTrain.RightBack.getCurrentPosition());
            MyOdometry.setAll(odometryRight, odometryLeft, odometryHorizental, currentTime);
            Xvelocity  = MyOdometry.getVelocityX();
            Yvelocity = MyOdometry.getVelocityY();
            if(MaxVelocityY < Yvelocity)
                MaxVelocityY = Yvelocity;
            if (MaxVelocityX < Xvelocity)
                MaxVelocityX = Xvelocity;
            packet.put("Yvelocity", Yvelocity);
            packet.put("Xvelocity", Xvelocity);
            double OdometryAngle = MyOdometry.getDirection();
            OurPoint position = MyOdometry.getPosition();
            packet.put("x:" , position.getX());
            packet.put("y:" , position.getY());
            packet.addLine("dAngle: " + OdometryAngle);
            double x = position.getX()*100/2.54;
            double y = position.getY()*100/2.54;
            double tmpX = x;
            double cosA = Math.cos(Math.toRadians(-90));
            double sinA = Math.sin(Math.toRadians(-90));
            x = tmpX*cosA - y*sinA;
            y = tmpX*sinA + y*cosA;
            packet.fieldOverlay().setStrokeWidth(1)
                    .setStroke("goldenrod")
                    .strokeCircle(x,y,0.19*100/2.54);

            dashboard.sendTelemetryPacket(packet);
        }
        MyDriveTrain.stop();
*/
        while (opModeIsActive()){
            packet = new TelemetryPacket();
            odometryRight = IntakeR.getCurrentPosition();
            odometryLeft = (IntakeL.getCurrentPosition());
            odometryHorizental = -(rightLinearMotor.getCurrentPosition());
            packet.put("odometryRight: ", odometryRight);
            packet.put("odometryLeft: ", odometryLeft);
            packet.put("odometryHorizental: ", odometryHorizental);
            updateOdometry();
            double OdometryAngle = MyOdometry.getDirection();
            OurPoint position = MyOdometry.getPosition();
            packet.put("OdometryAngle:" , OdometryAngle);
            packet.put("position:" , position.toString());
            packet.put("xRobot:" , MyOdometry.xRobot);
            packet.put("yRobot:" , MyOdometry.yRobot);
            packet.put("RminusL:" , MyOdometry.RminusL);
            packet.put("Change in angle:" , MyOdometry.DeltaAngle);
            packet.put("x:" , position.getX());
            packet.put("y:" , position.getY());
            double x = position.getX()*100/2.54;
            double y = position.getY()*100/2.54;
            double tmpX = x;
            double cosA = Math.cos(Math.toRadians(-90));
            double sinA = Math.sin(Math.toRadians(-90));
            x = tmpX*cosA - y*sinA;
            y = tmpX*sinA + y*cosA;
            packet.fieldOverlay().setStrokeWidth(1)
                                .setStroke("goldenrod")
                                .strokeCircle(x,y,0.225*100/2.54);

            //Set Drive Mode
            if (gamepad1.x) MyDriveTrain.setMode("arcade");
            else if (gamepad1.b) MyDriveTrain.setMode("Oriented");
            double direction = angles.firstAngle;
            packet.put("Mode: " , MyDriveTrain.Mode);
/*
            packet.put("RF", MyDriveTrain.RightFront.getPower());
            packet.put("LF", MyDriveTrain.LeftFront.getPower());
            packet.put("RB", MyDriveTrain.RightBack.getPower());
            packet.put("LB", MyDriveTrain.LeftBack.getPower());
*/

            //Drive by game pad
            if (MyDriveTrain.getMode().equals("Oriented")) {
                MyDriveTrain.fieldOriented(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, direction);
            }
            else {
                MyDriveTrain.arcade(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            /*
            if (gamepad1.left_bumper && Arm.getPosition() > 0.75) {
                Arm.setPosition(Arm.getPosition() - servoPosition);
            } else if (gamepad1.right_bumper && Arm.getPosition() < 1) {
                Arm.setPosition(Arm.getPosition() + servoPosition);
            } else {
                Arm.setPosition(Arm.getPosition());
            }

            telemetry.addData("Arm Position", Arm.getPosition());
            telemetry.update();

            if (gamepad1.a) {
                Output.setPosition(1);

            } else if (gamepad1.y) {
                Output.setPosition(0);
            }

            if (gamepad1.dpad_up) {

                LinearMotor.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                LinearMotor.setPower(-0.5);
            } else {
                LinearMotor.setPower(0);
            }
            if (gamepad1.dpad_right) {
                MyIntake.maxIntake();
            } else if (gamepad1.dpad_left) {
                MyIntake.maxOuttake();
            } else {
                MyIntake.move(0, 0);
            }
            */
            dashboard.sendTelemetryPacket(packet);
        }

    }


    }










