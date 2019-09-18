package org.firstinspires.ftc.teamcode;
import java.io.File;
import java.io.FileReader;
import java.io.BufferedReader;
import java.io.IOException;

public class PurePursuitGUI extends PathFollowerMecanum{


    //constants of PathFollower:

    //constractors:

    public PurePursuitGUI(OurPoint RobotPosition, double robotDirection, double targetDirection, double lookAheadDistance, double turnSpeed, double MaxAcceleration, double Kv, double Ka, double Kp, double Ki, double Kd, Odometry MyOdometry){
        super(RobotPosition, robotDirection, null , lookAheadDistance, targetDirection, MaxAcceleration, turnSpeed,Kv, Ka, Kp, Ki, Kd);;
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









}

