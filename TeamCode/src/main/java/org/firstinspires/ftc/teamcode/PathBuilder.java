package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class PathBuilder {



    public PathBuilder(double MaxVelocity, double MaxAcceleration, double Kc){

        //constants of PathGenerator:
        OurPoint[] Path = {
                new OurPoint(0.2, 0.2),
                new OurPoint(0.2, 1.2),
                new OurPoint(1.2, 1.2)};

        double maxSpace = 0.05;

        double weightData = 0.25;
        double weightSmooth = 0.75;
        double tolerance = 17;


        PathGenerator MyPathGenerator = new PathGenerator(Path, maxSpace, weightData, weightSmooth, tolerance, MaxVelocity, Kc, MaxAcceleration);
        Object[][] wayPoint = MyPathGenerator.getWayPoint();
        writeWayPoint(wayPoint);

    }

    private void writeWayPoint(Object[][] wayPoint){
        File csvFile = new File("wayPoint.csv");
        try {
            FileWriter writer = new FileWriter(csvFile);
            for(int i = 0; i < 3; i++){
                if(i == 0){
                    writer.append("x");
                    writer.append(",");
                }
                else if(i == 1){
                    writer.append("y");
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
