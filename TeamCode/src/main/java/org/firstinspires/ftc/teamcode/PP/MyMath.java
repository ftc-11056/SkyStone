package org.firstinspires.ftc.teamcode.PP;

import org.firstinspires.ftc.teamcode.PP.OurPoint;

public class MyMath {

    public static double t1 = 0;
    public static double t2 = 0;

    public static void main (String[] args){
        try {
            OurPoint p = intersectionPoint(new OurPoint(-0.189, -1.61), new OurPoint(-0.189, 1.56), new OurPoint(-0.19, -1.61), 0.19);
        }
        catch(Exception e){}
        //System.out.println(p.toString());
        System.out.println(t1);
        System.out.println(t2);
    }


    //methodes:

    public static double AngleMap(double angle){
        if (angle < 0){
            angle += 2*Math.PI;
        }
        if (angle > (2*Math.PI)){
            angle -= 2*Math.PI;
        }
        return angle;
    }

    public static double distance(OurPoint p1, OurPoint p2) {
        return Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
    }

    public static OurPoint calculateCenterPoint(OurPoint Q, OurPoint P, OurPoint R) {
        if (P.getX() == Q.getX()) {
            Q.setX(Q.getX() + 0.001);
        }
        if(P.getY() == Q.getY()){
            Q.setY(Q.getY() + 0.001);
        }
        double k1 = 0.5 * (square(P.getX()) + square(P.getY()) - square(Q.getX()) - square(Q.getY())) / (P.getX() - Q.getX());
        double k2 = (P.getY() - Q.getY()) / (P.getX() - Q.getX());
        double y = 0.5 * (square(Q.getX()) - 2 * Q.getX() * k1 + square(Q.getY()) - square(R.getX()) + 2 * R.getX() * k1 - square(R.getY())) / (R.getX() * k2 - R.getY() + Q.getY() - Q.getX() * k2);
        double x = k1 - k2 * y;
        return new OurPoint(x,y);
    }

    public static double square(double number) {
        return Math.pow(number, 2);
    }

    public static OurPoint intersectionPoint(OurPoint E, OurPoint L, OurPoint C, double r){
        OurPoint IntersectionPoint = null;
        OurPoint D = new OurPoint((L.getX() - E.getX()), (L.getY() - E.getY()));
        System.out.println(D.toString());
        OurPoint F = new OurPoint((E.getX() - C.getX()), (E.getY() - C.getY()));
        double a = DotProduct(D, D);
        double b = 2*DotProduct(F, D);
        double c = DotProduct(F, F) - square(r);
        Double discriminant = square(b) - 4*a*c;
        if(discriminant < 0){
            throw new RuntimeException("There is no intersection1");
        }
        else{
            discriminant = Math.sqrt(discriminant);
            t1 = (-b - discriminant)/(2*a);
            t2 = (-b + discriminant)/(2*a);
            if(t1 >= 0 && t1 <= 1){
                IntersectionPoint = new OurPoint((E.getX() + D.getX()*t1), (E.getY() + D.getY()*t1));
            }
            else if(t2 >= 0 && t2 <= 1){
                IntersectionPoint = new OurPoint((E.getX() + D.getX()*t2), (E.getY() + D.getY()*t2));
            }
            else{
                throw new RuntimeException("There is no intersection2");
            }
        }

        return IntersectionPoint;
    }

    public static double DotProduct(OurPoint p1, OurPoint p2){
        return p1.getX()*p2.getX() + p1.getY()*p2.getY();
    }

    public static double distanceBetweenLineAndPoint(double a, double b, double c, OurPoint p){
        return Math.abs(a*p.getX() + b*p.getY() + c)/Math.sqrt(square(a) + square(b));
    }

    public static double CrossProduct(OurPoint p1, OurPoint p2, OurPoint p3, OurPoint p4){
        return (p2.getY() - p1.getY())*(p4.getX() - p3.getX()) - (p2.getX() - p1.getX())*(p4.getY() - p3.getY());
    }

}
