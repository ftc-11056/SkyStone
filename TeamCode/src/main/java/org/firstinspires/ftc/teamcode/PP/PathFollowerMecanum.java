package org.firstinspires.ftc.teamcode.PP;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PP.OurPoint;

public class PathFollowerMecanum {

    //fields:

    public OurPoint RobotPosition;
    protected double robotDirection;
    protected Object[][] wayPoint;
    private int lustClosetPointIndex;
    protected final double lookAheadDistance;
    private double MaxAcceleration;
    private double lustOutputRateLimiter;
    private double lustTimeRateLimiter;
    private double closedPointBeforIndex;
    public OurPoint LookaheadPoint;
    protected double targetDirection;
    private final double turnSpeed;
    protected double targetVelocity;
    protected double measuredVelocity;

    protected double Xvelocity;
    protected double Yvelocity;
    protected double Cvelocity;
    protected double LookaheadPointAngleAccordingRobotLine;
    private double lastXtargetVelocity;
    private double lastYtargetVelocity;
    protected double XtargetAcceleration;
    protected double YtargetAcceleration;
    protected double Xdistance;

    private double Kv;
    private final double XencreaseConstant = 2;
    private double Ka;
    private double Kp;
    private final double Ki;
    private final double Kd;

    private double lastVelociityErrorX;
    private double lastVelociityErrorY;
    private double integralX;
    private double integralY;
    protected double Xpower;
    protected double Ypower;
    protected double Cpower;
    public boolean lost;
    public boolean stop;
    public double temp;
    public OurPoint temp1;
    protected boolean front;
    //constractors:

    public PathFollowerMecanum(OurPoint RobotPosition, Object[][] wayPoint, double lookAheadDistance, double MaxAcceleration, double turnSpeed, double Kv, double Ka, double Kp, double Ki, double Kd, boolean front){
        this.front = front;
        Xdistance = 0;
        LookaheadPointAngleAccordingRobotLine = 0;
        this.stop = false;
        this.lost = false;
        this.RobotPosition = RobotPosition;
        this.robotDirection = RobotPosition.getRadAngle();
        this.wayPoint = wayPoint;
        lustClosetPointIndex = 0;
        closedPointBeforIndex = 0;
        this.lookAheadDistance = lookAheadDistance;
        LookaheadPoint = null;
        this.MaxAcceleration = MaxAcceleration;
        this.lustOutputRateLimiter = 0;
        lustTimeRateLimiter = 0;
        this.targetDirection = 0;
        targetVelocity = 0;
        measuredVelocity = 0;
        Xvelocity = 0;
        Yvelocity = 0;
        lastXtargetVelocity = 0;
        lastYtargetVelocity = 0;
        XtargetAcceleration = 0;
        YtargetAcceleration = 0;
        this.Kv = Kv;
        this.turnSpeed = turnSpeed;
        this.Ka = Ka;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        lastVelociityErrorX = 0;
        lastVelociityErrorY = 0;
        integralX = 0;
        integralY = 0;
        Xpower = 0;
        Ypower = 0;
        Cpower = 0;
    }

    //methodes:

    public void setWayPoint(Object[][] wayPoint){
        this.wayPoint = wayPoint;
    }

    public int findClosetPointIndex(){
        double minDistance = Double.MAX_VALUE;
        int closetPointIndex = lustClosetPointIndex;
        for(int i = lustClosetPointIndex; i < wayPoint.length; i++){
            double newDistance = MyMath.distance(RobotPosition, (OurPoint) wayPoint[i][0]);
            if(newDistance < minDistance){
                minDistance = newDistance;
                closetPointIndex = i;
            }
        }
        lustClosetPointIndex = closetPointIndex;
        return closetPointIndex;
    }

    private int findIndexOfClosetPointOutOfCircle(){
        OurPoint EndPoint = (OurPoint) wayPoint[wayPoint.length-1][0];
        double distance = MyMath.distance(RobotPosition, EndPoint);
        if(distance < lookAheadDistance){
            return -1;
        }
        boolean stop = false;
        int i = wayPoint.length - 2;
        while(i > 0 && !stop){
            distance = MyMath.distance(RobotPosition, (OurPoint) wayPoint[i][0]);
            if(distance < lookAheadDistance) {
                stop = true;
            }
            i--;
        }
        return (i + 2);
    }

    private OurPoint findLookaheadPoint(){
        int closetPointOutOfCircleIndex = findIndexOfClosetPointOutOfCircle();
        if(closetPointOutOfCircleIndex == -1){
            return (OurPoint) wayPoint[wayPoint.length - 1][0];
        }
        OurPoint E = (OurPoint)wayPoint[closetPointOutOfCircleIndex - 1][0];
        OurPoint L = (OurPoint)wayPoint[closetPointOutOfCircleIndex][0];
        OurPoint C = RobotPosition;
        OurPoint intersection = null;
        try{
            intersection = MyMath.intersectionPoint(E, L, C, lookAheadDistance);
        }
        catch (Exception e){

        }
        return intersection;
    }

    private double calculateHorizontalDistance(OurPoint LookaheadPoint){
        double a = -Math.tan(robotDirection);
        double b = 1;
        double c = Math.tan(robotDirection)*RobotPosition.getX()- RobotPosition.getY();
        double x = MyMath.distanceBetweenLineAndPoint(a, b, c, LookaheadPoint);
        return x;
    }

    private double side(OurPoint LookaheadPoint){
        OurPoint PointOnRobotLine = new OurPoint(RobotPosition.getX() + Math.cos(robotDirection), RobotPosition.getY() + Math.sin(robotDirection));
        double crossProduct = MyMath.CrossProduct(RobotPosition, PointOnRobotLine, RobotPosition, LookaheadPoint);
        if(crossProduct >= 0 )
            return 1;
        else
            return -1;
    }

    private void UpdateVelocitiesByRobotPosition(TelemetryPacket packet,double time, OurPoint RobotPosition){
        this.RobotPosition = RobotPosition;
        this.robotDirection = RobotPosition.getRadAngle();
        int closetPointIndex = findClosetPointIndex();
        temp1 = (OurPoint)wayPoint[closetPointIndex][0];
        temp = ((OurPoint)wayPoint[closetPointIndex][0]).getRadAngle();
        targetDirection = (((OurPoint)wayPoint[closetPointIndex][0]).getRadAngle()) + (Math.PI / 2);
        LookaheadPoint = findLookaheadPoint();
        if(LookaheadPoint == null) {
            lost = true;
            stop = true;
            return;
        }
        if(closetPointIndex == wayPoint.length-1){
            stop = true;
            return;
        }
        Xdistance = side(LookaheadPoint)*calculateHorizontalDistance(LookaheadPoint);
        LookaheadPointAngleAccordingRobotLine = Math.acos(Xdistance / lookAheadDistance);
        if(!front){
             LookaheadPointAngleAccordingRobotLine *= -1;
        }
        targetVelocity = (Double)wayPoint[closetPointIndex][1];
        targetVelocity = rateLimiter(targetVelocity, time);
        Xvelocity = targetVelocity * Math.cos(LookaheadPointAngleAccordingRobotLine) * XencreaseConstant;
        Yvelocity = targetVelocity * Math.sin(LookaheadPointAngleAccordingRobotLine);
        XtargetAcceleration = (Xvelocity - lastXtargetVelocity) / time;
        YtargetAcceleration = (Yvelocity - lastYtargetVelocity) / time;
        lastXtargetVelocity = Xvelocity;
        lastYtargetVelocity = Yvelocity;
        Cvelocity = Range.clip((robotDirection - (targetDirection))/ Math.toRadians(30), -1, 1);
    }

    public double rateLimiter(double targetVelocity, double time){
        double MaxChange = (time - lustTimeRateLimiter)*MaxAcceleration;
        double output = lustOutputRateLimiter;
        output += constrain((targetVelocity - lustOutputRateLimiter), -MaxChange, MaxChange);
        lustOutputRateLimiter = output;
        lustTimeRateLimiter = time;
        return output;
    }

//    public static void main(String[] args){
//        PurePursuitGUI p = new PurePursuitGUI();
//        for(int i = 1; i < 10; i++)
//            System.out.println(p.rateLimiter(1.5, 0.050*i));
//    }

    private double constrain(double value, double minLimit, double maxLimit){
        if(value > maxLimit){
            return maxLimit;
        }
        else if(value < minLimit){
            return minLimit;
        }
        else{
            return value;
        }
    }

    private double PID(double measuredVelocity, String axis){
        double PID = 0;
        if(axis.equals("X")){
            double error = lastXtargetVelocity - measuredVelocity;
            integralX += error;
            double derivative = error - lastVelociityErrorX;
            PID = Kp*error + Ki* integralX + Kd*derivative;
            lastVelociityErrorX = error;
        }
        if(axis.equals("Y")){
            double error = lastYtargetVelocity - measuredVelocity;
            integralY += error;
            double derivative = error - lastVelociityErrorY;
            PID = Kp*error + Ki* integralY + Kd*derivative;
            lastVelociityErrorY = error;
        }
        return PID;
    }

    public void UpdatePowerByRobotPosition(TelemetryPacket packet, double time, OurPoint RobotPosition, double measuredVelocityX, double measuredVelocityY){
        UpdateVelocitiesByRobotPosition(packet, time, RobotPosition);
        measuredVelocity = Math.sqrt(MyMath.square(measuredVelocityX) + MyMath.square(measuredVelocityY));
        double FeedForwardX = Kv*Xvelocity + Ka*XtargetAcceleration;
        double FeedForwardY = Kv*Yvelocity + Ka*YtargetAcceleration;
        double FeedForwardC = Cvelocity * turnSpeed;
        double FeedBackX = PID(measuredVelocityX, "X");
        double FeedBackY = PID(measuredVelocityY, "Y");
        packet.put("FF y", FeedForwardY);
        packet.put("FB y", FeedBackY);
        packet.put("acceleration factor y", Ka*YtargetAcceleration);
        packet.put("Velocity factor y", Kv*Yvelocity);
        Xpower = FeedForwardX + FeedBackX;
        Ypower = FeedForwardY + FeedBackY;
        Cpower = FeedForwardC;
    }

    public double getXpower() {
        return Xpower;
    }

    public double getYpower() {
        return Ypower;
    }

    public double getCpower() {
        return Cpower;
    }
    public void setKa(double Ka){
        this.Ka = Ka;
    }
    public void setKp(double Kp){
        this.Kp = Kp;
    }
    public void setKv(double Kv){
        this.Kv = Kv;
    }
}
