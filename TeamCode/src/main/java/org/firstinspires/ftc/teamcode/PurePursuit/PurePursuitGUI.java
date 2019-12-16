package org.firstinspires.ftc.teamcode.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Systems.Odometry;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

@Config
public class PurePursuitGUI extends PathFollowerMecanum {


    //Fields:
    public static double Xstart = 0;
    public static double Ystart = 0;
    private static OurPoint StartRobotPosition = new OurPoint(Xstart,Ystart);
    public static double startRobotDirection = 0;
    public static double MaxVelocity = 1.727875947;
    public static double MaxAcceleration = 2;
    public static double turnSpeed = 0.75;
    public static double targetDirection = 0;
    public static double Kc = 2;
    public static double lookAheadDistance = 0.2;
    public static double Kv = 1/ MaxVelocity;
    public static double Ka = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;

    private FtcDashboard dashboard;
    private PathBuilder MyPathBuilder;

    //constructors:

    public PurePursuitGUI(FtcDashboard dashboard){
        super(StartRobotPosition, startRobotDirection, null , lookAheadDistance, targetDirection, MaxAcceleration, turnSpeed,Kv, Ka, Kp, Ki, Kd);
        MyPathBuilder = new PathBuilder(MaxVelocity, MaxAcceleration, Kc);
        readWayPointFromCSV();
        this.dashboard = dashboard;
    }

    //methodes:

    private void readWayPointFromCSV(){
        Object[][] wayPoint = null;
        try {
            FileReader FR = new FileReader("wayPoint.csv");
            BufferedReader csvReader = new BufferedReader(FR);
            String row = csvReader.readLine();
            String[] line1 = row.split(",");
            row = csvReader.readLine();
            String[] line2 = row.split(",");
            row = csvReader.readLine();
            String[] line3 = row.split(",");

            wayPoint = new Object[line1.length-1][2];
            for(int i = 1; i < wayPoint.length+1; i++){
                wayPoint[i-1][0] = new OurPoint(Double.parseDouble(line1[i]), Double.parseDouble(line2[i]));
                wayPoint[i-1][1] = Double.parseDouble(line3[i]);
            }

            csvReader.close();
        }
        catch (IOException e) {

        }
        setWayPoint(wayPoint);
    }


    public void updateGraghic(){
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Velocity", targetVelocity);
        packet.put("Measured Velocity", measuredVelocity);
        packet.put("X Velocity",Xvelocity);
        packet.put("Y Velocity",Yvelocity);
        packet.put("C Velocity",Cvelocity);
        double[][] path = new  double[2][wayPoint.length];
        for(int i = 0; i < wayPoint.length; i++){
            path[0][i] = ((OurPoint)wayPoint[i][0]).getX()*100/2.54;
            path[1][i] = ((OurPoint)wayPoint[i][0]).getY()*100/2.54;
        }
        turnCoordinateSystem(path[0], path[1], -90);
        packet.fieldOverlay().setStrokeWidth(1)
                             .setStroke("goldenrod")
                             .strokePolyline(path[0], path[1])
                             .strokeCircle(RobotPosition.getX(), RobotPosition.getY(), lookAheadDistance);
        dashboard.sendTelemetryPacket(packet);
    }

    public Odometry buildOdometry(){
        return new Odometry(StartRobotPosition, startRobotDirection);
    }

    private void turnCoordinateSystem(double[] x, double[] y, double turnAngle){
        double cosA = Math.cos(Math.toRadians(turnAngle));
        double sinA = Math.sin(Math.toRadians(turnAngle));
        for(int i = 0; i < x.length; i++){
            double tempx = x[i];
            x[i] = x[i]*cosA - y[i]*sinA;
            y[i]= tempx*sinA + y[i]*cosA;
        }
    }






}

