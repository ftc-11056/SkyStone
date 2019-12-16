package org.firstinspires.ftc.teamcode.PurePursuit;

public class OurPoint {

    //fields:

    private double x;
    private double y;


    //constractors:

    public OurPoint(double x, double y){
        this.x = x;
        this.y = y;
    }

    public OurPoint(){
        this.x = 0;
        this.y = 0;
    }


    //methodes:

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void move(double dX, double dY){
        x += dX;
        y += dY;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
