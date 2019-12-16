package org.firstinspires.ftc.teamcode.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Sine wave sample to demonstrate telemetry and config variables in action. Adjust the amplitude,
 * phase, and frequency of the oscillation and watch the changes propagate immediately to the graph.
 */
@Disabled
@Config
@Autonomous
public class SineWaveOpMode extends LinearOpMode {
    public static double AMPLITUDE = 10;
    public static double PHASE = 90;
    public static double FREQUENCY = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();
        TelemetryPacket packet = new TelemetryPacket();
        TelemetryPacket packet2 = new TelemetryPacket();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            packet.put("TargetVel", AMPLITUDE * Math.sin(
                    2 * Math.PI * FREQUENCY * getRuntime() + Math.toRadians(PHASE)
            ));
            packet.put("MeasuredVel", AMPLITUDE * Math.cos(
                    2 * Math.PI * FREQUENCY * getRuntime() + Math.toRadians(PHASE)
            ));

            dashboard.sendTelemetryPacket(packet);
        }
    }

}