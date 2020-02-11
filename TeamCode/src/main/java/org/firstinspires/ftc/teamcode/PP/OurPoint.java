package org.firstinspires.ftc.teamcode.PP;

public class OurPoint {

    //fields:

    private double x;
    private double y;
    private double angle;

    //constractors:

    public OurPoint(double x, double y) {
        this.x = x;
        this.y = y;
        this.angle = 0;
    }

    public OurPoint() {
        this.x = 0;
        this.y = 0;
        this.angle = 0;
    }

    public OurPoint(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public OurPoint(OurPoint p){
        x = p.getX();
        y = p.getY();
        angle = p.getDegAngle();
    }


    //methodes:

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getDegAngle() {
        return angle;
    }

    public double getRadAngle() {
        return Math.toRadians(angle);
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setDegAngle(double angle) {
        this.angle = angle;
    }

    public void setRadAngle(double angle) {
        this.angle = Math.toDegrees(angle);
    }

    public void move(double dX, double dY) {
        x += dX;
        y += dY;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + angle + ")";
    }
}
