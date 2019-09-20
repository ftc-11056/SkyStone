package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class PurePursuitGUI extends PathFollowerMecanum{


    //Fields:
    private FtcDashboard dashboard;


    //constructors:

    public PurePursuitGUI(OurPoint RobotPosition, double robotDirection, double targetDirection, double lookAheadDistance, double turnSpeed, double MaxAcceleration, double Kv, double Ka, double Kp, double Ki, double Kd, Odometry MyOdometry){
        super(RobotPosition, robotDirection, null , lookAheadDistance, targetDirection, MaxAcceleration, turnSpeed,Kv, Ka, Kp, Ki, Kd);;
        readWayPointFromCSV();
        dashboard = FtcDashboard.getInstance();
    }

    //methodes:

    private void readWayPointFromCSV(){
        Object[][] wayPoint = null;
        String csvAdress = "C:\\ybot\\Program2020\\sky-stone";
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


    public void updateGraghics(){
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", targetVelocity);
        packet.put("x", measuredVelocity);
        dashboard.sendTelemetryPacket(packet);
    }






}

