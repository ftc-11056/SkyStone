package org.firstinspires.ftc.teamcode.PP;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PP.OurPoint;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class PathBuilder {

    public StringBuilder SB = null;



    public static void main(String[] args){
        OurPoint[] Path4 = {
                new OurPoint(0.74, 1.15, 270),
                new OurPoint(1, 1.15,180),
                new OurPoint(1, 0.40, 180),
                new OurPoint(0.9, -0.85, 130),
                new OurPoint(0.55, -1.35, 130),
                new OurPoint(0.36, -1.38, 130)};
        PathBuilder bf = new PathBuilder(Path4,1.5, 1.2, 5, 82);
    }



    public PathBuilder(OurPoint[] Path, double MaxVelocity, double MaxAcceleration, double Kc, double tolerance){

        //constants of PathGenerator:

        double maxSpace = 0.05;

        double weightData = 0.1;
        double weightSmooth = 1 - weightData;


        System.out.println("1");
        PathGenerator MyPathGenerator = new PathGenerator(Path, maxSpace, weightData, weightSmooth, tolerance, MaxVelocity, Kc, MaxAcceleration);
        System.out.println("2");
        Object[][] wayPoint = MyPathGenerator.getWayPoint();
        writeWayPointToPhone(wayPoint);
        System.out.println("3");
    }

    private void writeWayPointToPhone(Object[][] wayPoint){
        File csvFile = AppUtil.getInstance().getSettingsFile("wayPoint.csv");
        SB = new StringBuilder();
        for(int i = 0; i < 4; i++) {
            if (i == 0) {
                SB.append("x");
                SB.append(",");
            }
            else if (i == 1) {
                SB.append("y");
                SB.append(",");
            }
            else if(i == 2){
                SB.append("Angle");
                SB.append(",");
            }
            else {
                SB.append("velocity");
                SB.append(",");
            }
            for (int j = 0; j < wayPoint.length; j++) {
                double dValue = 0;
                if (i == 0) {
                    dValue = ((OurPoint) wayPoint[j][0]).getX();
                } else if (i == 1) {
                    dValue = ((OurPoint) wayPoint[j][0]).getY();
                }
                else if(i==2){
                    dValue = ((OurPoint) wayPoint[j][0]).getDegAngle();
                }
                else {
                    dValue = (Double) wayPoint[j][3];
                }

                String sValue = String.valueOf(dValue);
                SB.append(sValue);
                SB.append(",");
            }
            SB.append("\n");
        }
        ReadWriteFile.writeFile(csvFile, SB.toString());
    }

    private void writeWayPointToComputer(Object[][] wayPoint){
        File csvFile = new File("wayPoint.csv");
        try {
            FileWriter writer = new FileWriter(csvFile);
            for(int i = 0; i < 4; i++){
                if(i == 0){
                    writer.append("x");
                    writer.append(",");
                }
                else if(i == 1){
                    writer.append("y");
                    writer.append(",");
                }
                else if(i == 2){
                    writer.append("Angle");
                    writer.append(",");
                }
                else{
                    writer.append("velocity");
                    writer.append(",");
                }
                for(int j = 0; j < wayPoint.length; j++){
                    double dValue = 0;
                    if(i == 0){
                        dValue = ((OurPoint) wayPoint[j][0]).getX();
                    }
                    else if(i == 1){
                        dValue =  ((OurPoint) wayPoint[j][0]).getY();
                    }
                    else if(i == 2){
                        dValue =  ((OurPoint) wayPoint[j][0]).getRadAngle();
                    }
                    else{
                        dValue =  (Double)wayPoint[j][3];
                    }

                    String sValue = String.valueOf(dValue);
                    writer.append(sValue);
                    writer.append(",");
                }
                writer.append("\n");

            }
            writer.flush();
            writer.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println("Build Path");
    }



}
