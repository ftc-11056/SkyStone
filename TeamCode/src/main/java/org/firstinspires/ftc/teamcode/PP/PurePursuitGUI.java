package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PP.OurPoint;
import org.firstinspires.ftc.teamcode.PP.PathBuilder;
import org.firstinspires.ftc.teamcode.PP.PathFollowerMecanum;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

@Config
public class PurePursuitGUI extends PathFollowerMecanum {


    //Fields:
    public static double MaxAcceleration = 1.2;
    public static double lookAheadDistance = 0.225;
    public static double Ka = 0.8;
    public static double Kp = 0.3;
    public static double Ki = 0;
    public static double Kd = 0;

    private PathBuilder MyPathBuilder;

    //constructors:

    public PurePursuitGUI(OurPoint[] Path, OurPoint Position, double tolerance, double Kc, double MaxVelocity, double turnSpeed, boolean front){
        super(Position, null , lookAheadDistance, MaxAcceleration, turnSpeed,1 / MaxVelocity, Ka, Kp, Ki, Kd, front);
        MyPathBuilder = new PathBuilder(Path, MaxVelocity, MaxAcceleration, Kc, tolerance);
        readWayPointFromCSV();
    }

    //methodes:

    private void readWayPointFromCSV(){
        Object[][] wayPoint = null;
        try {
//            FileReader FR = new FileReader("wayPoint.csv");
            File csv = AppUtil.getInstance().getSettingsFile("wayPoint.csv");
            FileReader FR = new FileReader(csv);
            BufferedReader csvReader = new BufferedReader(FR);
            String row = csvReader.readLine();
            String[] line1 = row.split(",");
            row = csvReader.readLine();
            String[] line2 = row.split(",");
            row = csvReader.readLine();
            String[] line3 = row.split(",");
            row = csvReader.readLine();
            String[] line4 = row.split(",");
            wayPoint = new Object[line1.length-1][2];

            for(int i = 1; i < wayPoint.length+1; i++){
                wayPoint[i-1][0] = new OurPoint(Double.parseDouble(line1[i]), Double.parseDouble(line2[i]),Double.parseDouble(line3[i]));
                wayPoint[i-1][1] = Double.parseDouble(line4[i]);
            }

            csvReader.close();
        }
        catch (IOException e) {

        }
        setWayPoint(wayPoint);
    }

    private OurPoint turnRobotPosition(OurPoint position){
        double x = position.getX()*100/2.54;
        double y = position.getY()*100/2.54;
        double tmpX = x;
        double cosA = Math.cos(Math.toRadians(-90));
        double sinA = Math.sin(Math.toRadians(-90));
        x = tmpX*cosA - y*sinA;
        y = tmpX*sinA + y*cosA;
        return new OurPoint(x,y);
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

    public void updateGraghic(TelemetryPacket packet){
        packet.put("Target Velocity", targetVelocity);
        packet.put("Measured Velocity", measuredVelocity);
        packet.put("X Power",Xpower);
        packet.put("Y Power",Ypower);
        packet.put("C Power",Cpower);
        packet.put("Direction",robotDirection);
        packet.put("Target Direction",targetDirection);
        packet.put("Point angle",temp);
        packet.put("Robot position", RobotPosition.toString());
        if(temp1 != null){
            packet.put("closed Point", temp1.toString());
        }
        packet.put("Robot - point - angle", LookaheadPointAngleAccordingRobotLine);
        double[][] path = new  double[2][wayPoint.length];
        for(int i = 0; i < wayPoint.length; i++){
            path[0][i] = ((OurPoint)wayPoint[i][0]).getX()*100/2.54;
            path[1][i] = ((OurPoint)wayPoint[i][0]).getY()*100/2.54;
        }
        turnCoordinateSystem(path[0], path[1], -90);
        OurPoint FieldRobotPosition = turnRobotPosition(RobotPosition);
        double robotCircleRadius = lookAheadDistance*100/2.54;
        packet.fieldOverlay().setStrokeWidth(1)
                .setStroke("goldenrod")
                .strokePolyline(path[0], path[1])
                .strokeCircle(FieldRobotPosition.getX(), FieldRobotPosition.getY(), robotCircleRadius);
    }





}

