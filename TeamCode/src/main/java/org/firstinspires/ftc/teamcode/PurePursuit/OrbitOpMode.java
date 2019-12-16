package org.firstinspires.ftc.teamcode.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Demonstration of the dashboard's field overlay display capabilities.
 */
@Config
@Autonomous
@Disabled
public class OrbitOpMode extends LinearOpMode {
    public static double ORBITAL_FREQUENCY = 0.05;
    public static double SPIN_FREQUENCY = 0.25;

    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double time = getRuntime();

            double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
            double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);
            double l = SIDE_LENGTH / 2;

            double[] bxPoints = { l, -l, -l, l };
            double[] byPoints = { l, l, -l, -l };
            rotatePoints(bxPoints, byPoints, 2 * Math.PI * SPIN_FREQUENCY * time);
            for (int i = 0; i < 4; i++) {
                bxPoints[i] += bx;
                byPoints[i] += by;
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("TargetVel", 10 * Math.sin(2 * Math.PI * 0.5 * getRuntime() + Math.toRadians(90)));
            packet.put("x", bxPoints[0]);
            packet.put("y", byPoints[0]);
            double[] x = {0.2*100/2.54,0.2*100/2.54,1.2*100/2.54};
            double[] y = {0.2*100/2.54,1.2*100/2.54,1.2*100/2.54};
            turnCoordinateSystem(x,y);
            packet.fieldOverlay()
                    .setStrokeWidth(1)
                    .setStroke("goldenrod")
                    .strokePolyline(x,y)
                    .setFill("black")
                    .fillPolygon(bxPoints, byPoints);
            dashboard.sendTelemetryPacket(packet);
        }

    }

    private void turnCoordinateSystem(double[] x, double[] y){
        double cosA = Math.cos(Math.toRadians(-90));
        double sinA = Math.sin(Math.toRadians(-90));
        for(int i = 0; i < x.length; i++){
            double tempx = x[i];
            x[i] = x[i]*cosA - y[i]*sinA;
            y[i]= tempx*sinA + y[i]*cosA;
        }

    }
}