package org.firstinspires.ftc.teamcode.PP;

public class Path {

    //fields:

    private OurPoint[] WayPoints;
    private double tolerance;
    private double Kc;
    private double MaxVelocity;
    private double turnSpeed;
    private boolean front;

    //constractors:

    public Path (OurPoint[] WayPoints, double tolerance, double Kc, double MaxVelocity, double turnSpeed, boolean front){
        this.WayPoints = WayPoints;
        this.tolerance = tolerance;
        this.Kc = Kc;
        this.MaxVelocity = MaxVelocity;
        this.turnSpeed = turnSpeed;
        this.front = front;
    }


    //methodes:


    public boolean isFront() {
        return front;
    }

    public double getKc() {
        return Kc;
    }

    public double getMaxVelocity() {
        return MaxVelocity;
    }

    public double getTolerance() {
        return tolerance;
    }

    public double getTurnSpeed() {
        return turnSpeed;
    }

    public OurPoint[] getWayPoints() {
        return WayPoints;
    }
}
