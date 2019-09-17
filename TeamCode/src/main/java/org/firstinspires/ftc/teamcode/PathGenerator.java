package org.firstinspires.ftc.teamcode;

import java.util.Arrays;

public class PathGenerator {

//fields:

    private Object[][] wayPoint;
    private Object[][] curveCircle;
    private OurPoint[] path;
    private Double MaxAcceleration;


//constractors:


    public PathGenerator(OurPoint[] path, double maxSpace, double weightData, double weightSmooth, double tolerance, double MaxVelocity, double Kc, double MaxAcceleration) {
        this.path = path;
        injection(maxSpace);
        smoother(weightData, weightSmooth, tolerance);
        wayPoint = new Object[this.path.length][4];
        insertPath(this.path);
        setDistance();
        curveCircle = new Object[wayPoint.length][3];
        setCurvature();
        this.MaxAcceleration = MaxAcceleration;
        setVelocitys(MaxVelocity, Kc);
        printWayPoint();

    }

    private void printWayPoint(){
        for(int j = 0; j < 4; j++) {
            System.out.print("[");
            for (int i = 0; i < wayPoint.length; i++) {
                System.out.print(wayPoint[i][j]);
                if (i < wayPoint.length - 1) {
                    System.out.print(",");
                }
            }
            System.out.println("]");
        }
    }

    //Methodes:
    private void insertPath(OurPoint[] path) {
        for (int i = 0; i < path.length; i++) {
            wayPoint[i][0] = path[i];
        }
    }

    private void injection(double maxSpace) {
        double numToInject = numToInject(maxSpace);
        int numberOfPoints = 0;

        OurPoint[] temp = new OurPoint[(int)(numToInject + 1)];

        int index = 0;
        for (int i = 0; i < (path.length - 1); i++) {
            OurPoint p1 = path[i];
            OurPoint p2 = path[i + 1];
            double distance = MyMath.distance(p1, p2);
            numberOfPoints = (int)Math.ceil(distance / maxSpace);
            OurPoint vector = new OurPoint(((p2.getX() - p1.getX()) / numberOfPoints), ((p2.getY() - p1.getY()) / numberOfPoints));
            for (int j = 0; j < numberOfPoints; j++) {
                temp[index] = new OurPoint((p1.getX() + vector.getX() * j), (p1.getY() + vector.getY() * j));
                index++;
            }
        }
        temp[index] = path[path.length - 1];
        path = temp;
    }

    private int numToInject(double maxSpace) {
        int numberOfPoints = 0;
        for (int i = 0; i < (path.length - 1); i++) {
            double distance = MyMath.distance(path[i], path[i + 1]);
            numberOfPoints += Math.ceil(distance / maxSpace);
        }
        return numberOfPoints;
    }

    private void smoother(double weight_data, double weight_smooth, double tolerance) {

        //copy array
        OurPoint[] newPath = Arrays.copyOf(path, path.length);

        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < path.length - 1; i++) {
                double aux = newPath[i].getX();
                double dX = weight_data * (path[i].getX() - newPath[i].getX()) + weight_smooth * (newPath[i - 1].getX() + newPath[i + 1].getX() - (2.0 * newPath[i].getX()));
                double dY = weight_data * (path[i].getY() - newPath[i].getY()) + weight_smooth * (newPath[i - 1].getY() + newPath[i + 1].getY() - (2.0 * newPath[i].getY()));
                newPath[i].move(dX, dY);
                change += Math.abs(aux - newPath[i].getX());
                change += Math.abs(aux - newPath[i].getY());
            }
        }
        path = newPath;
    }

    private void setDistance(){
        wayPoint[0][1] = new Double (0);
        for (int i = 1; i < wayPoint.length; i++) {
            wayPoint[i][1] = (Double)wayPoint[i-1][1] + (MyMath.distance((OurPoint) wayPoint[i][0],(OurPoint)wayPoint[i-1][0]));
        }
    }

    private void setCurvature(){
        wayPoint[0][2] = new Double(0);

        curveCircle[0][0] = wayPoint[0][0];
        curveCircle[0][1] = null;
        curveCircle[0][2] = 0;

        for (int i = 1; i < wayPoint.length-1; i++) {
            OurPoint Q = (OurPoint) wayPoint[i-1][0];
            OurPoint P = (OurPoint) wayPoint[i][0];
            OurPoint R = (OurPoint) wayPoint[i+1][0];
            OurPoint centerCircle = MyMath.calculateCenterPoint(Q, P, R);
            double radius = MyMath.distance(P,centerCircle);
            curveCircle[i][0] = wayPoint[i][0];
            curveCircle[i][1] = centerCircle;
            curveCircle[i][2] = radius;
            wayPoint[i][2] = new Double(1/radius);
        }

        wayPoint[wayPoint.length-1][2] = new Double(0);

        curveCircle[curveCircle.length-1][0] = wayPoint[wayPoint.length-1][0];
        curveCircle[curveCircle.length-1][1] = null;
        curveCircle[curveCircle.length-1][2] = 0;

    }

    private void setVelocitys(double MaxVelocity, double k){
        for(int i = 0; i < wayPoint.length; i++){
            wayPoint[i][3] = Math.min(MaxVelocity, k / (Double)wayPoint[i][2]);
        }
        wayPoint[wayPoint.length-1][3] = new Double(0);
        for(int i = wayPoint.length-2; i >= 0; i--){
            double distance = (Double)wayPoint[i+1][1] - (Double)wayPoint[i][1];
            double descriminant = MyMath.square((Double)wayPoint[i+1][3]) + 2*MaxAcceleration*distance;
            double MaxVel = Math.sqrt(descriminant);
            wayPoint[i][3] = new Double(Math.min((Double)wayPoint[i][3], MaxVel));
        }
    }


    public Object[][] getWayPoint() {
        return wayPoint;
    }

    public Object[][] getCurveCircle() {
        return curveCircle;
    }
}
