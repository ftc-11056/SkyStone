package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class PathFollowerMecanum {

    //fields:

    private OurPoint RobotPosition;
    private double robotDirection;
    private Object[][] wayPoint;
    private int lustClosetPointIndex;
    private final double lookAheadDistance;
    private double MaxAcceleration;
    private double lustOutputRateLimiter;
    private double lustTimeRateLimiter;
    private double targetDirection;
    private final double turnSpeed;
    protected double targetVelocity;
    protected double measuredVelocity;

    private double Xvelocity;
    private double Yvelocity;
    private double Cvelocity;

    private double lastXtargetVelocity;
    private double lastYtargetVelocity;
    private double XtargetAcceleration;
    private double YtargetAcceleration;

    private final double Kv;
    private final double XdecreaseConstant = 1;
    private final double Ka;
    private final double Kp;
    private final double Ki;
    private final double Kd;

    private double lastVelociityErrorX;
    private double lastVelociityErrorY;
    private double integralX;
    private double integralY;
    private double Xpower;
    private double Ypower;
    private double Cpower;

    //constractors:

    public PathFollowerMecanum(OurPoint RobotPosition, double robotDirection, Object[][] wayPoint, double lookAheadDistance, double targetDirection, double MaxAcceleration, double turnSpeed, double Kv, double Ka, double Kp, double Ki, double Kd){
        this.RobotPosition = RobotPosition;
        this.robotDirection = robotDirection;
        this.wayPoint = wayPoint;
        lustClosetPointIndex = 0;
        this.lookAheadDistance = lookAheadDistance;
        this.MaxAcceleration = MaxAcceleration;
        this.lustOutputRateLimiter = 0;
        lustTimeRateLimiter = 0;
        this.targetDirection = Math.toRadians(targetDirection);
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

    }

    //methodes:

    public void setWayPoint(Object[][] wayPoint){
        this.wayPoint = wayPoint;
    }

    private int findClosetPointIndex(){
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

    private OurPoint findClosetPointBefore(){
        OurPoint before = (OurPoint) wayPoint[0][0];
        double minDistance = MyMath.distance(RobotPosition, before);
        boolean stop = false;
        for(int i = 1; i < wayPoint.length && !stop; i++){
            double newDistance = MyMath.distance(RobotPosition, (OurPoint) wayPoint[i][0]);
            if(newDistance < minDistance){
                before = (OurPoint) wayPoint[i][0];
                minDistance = newDistance;
            }
            else{
                stop = true;
            }
        }
        return before;
    }

    private OurPoint findClosetPointAfter(){
        OurPoint after = (OurPoint) wayPoint[wayPoint.length-1][0];
        double minDistance = MyMath.distance(RobotPosition, after);
        boolean stop = false;
        for(int i = wayPoint.length - 2; i >= 0 && !stop; i++){
            double newDistance = MyMath.distance(RobotPosition, (OurPoint) wayPoint[i][0]);
            if(newDistance < minDistance){
                after = (OurPoint) wayPoint[i][0];
                minDistance = newDistance;
            }
            else{
                stop = true;
            }
        }
        return after;
    }

    private OurPoint findLookaheadPoint(){
        OurPoint E = findClosetPointBefore();
        OurPoint L = findClosetPointAfter();
        OurPoint C = RobotPosition;
        OurPoint intersection = null;
        try{
            intersection = MyMath.intersectionPoint(E, L, C, lookAheadDistance);
        }
        catch(Exception e){
            //There is no intersection
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


    private void UpdateVelocitiesByRobotPosition(double time, OurPoint RobotPosition, double robotDirection){
        this.RobotPosition = RobotPosition;
        this.robotDirection = robotDirection;
        int closetPointIndex = findClosetPointIndex();
        OurPoint LookaheadPoint = findLookaheadPoint();
        double Xdistance = calculateHorizontalDistance(LookaheadPoint);
        double LookaheadPointAngleAccordingRobotLine = Math.acos(Xdistance / lookAheadDistance);
        targetVelocity = (Double)wayPoint[closetPointIndex][1];
        targetVelocity = rateLimiter(targetVelocity, time);
        Xvelocity = targetVelocity * Math.cos(LookaheadPointAngleAccordingRobotLine) * XdecreaseConstant;
        Yvelocity = targetVelocity * Math.sin(LookaheadPointAngleAccordingRobotLine);
        XtargetAcceleration = (Xvelocity - lastXtargetVelocity) / time;
        YtargetAcceleration = (Yvelocity - lastYtargetVelocity) / time;
        lastXtargetVelocity = Xvelocity;
        lastYtargetVelocity = Yvelocity;
        Cvelocity = Range.clip((LookaheadPointAngleAccordingRobotLine + targetDirection)/ Math.toRadians(30), -1, 1);
    }

    private double rateLimiter(double targetVelocity, double time){
        double MaxChange = (time - lustTimeRateLimiter)*MaxAcceleration;
        double output = lustOutputRateLimiter;
        output += constrain((targetVelocity - lustOutputRateLimiter), -MaxChange, MaxChange);
        lustOutputRateLimiter = output;
        lustTimeRateLimiter = time;
        return output;
    }

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

    private double PID(double measuredVelocity, String Direction){
        double PID = 0;
        if(Direction.equals("X")){
            double error = lastXtargetVelocity - measuredVelocity;
            integralX += error;
            double derivative = error - lastVelociityErrorX;
            PID = Kp*error + Ki* integralX + Kd*derivative;
            lastVelociityErrorX = error;
        }
        if(Direction.equals("Y")){
            double error = lastYtargetVelocity - measuredVelocity;
            integralY += error;
            double derivative = error - lastVelociityErrorY;
            PID = Kp*error + Ki* integralY + Kd*derivative;
            lastVelociityErrorY = error;
        }
        return PID;
    }

    public void UpdatePowerByRobotPosition(double time, OurPoint RobotPosition, double robotDirection, double measuredVelocityX, double measuredVelocityY){
        UpdateVelocitiesByRobotPosition(time, RobotPosition, robotDirection);
        measuredVelocity = Math.sqrt(MyMath.square(measuredVelocityX)+MyMath.square(measuredVelocityY));
        double FeedForwardX = Kv*Xvelocity + Ka*XtargetAcceleration;
        double FeedForwardY = Kv*Yvelocity + Ka*YtargetAcceleration;
        double FeedForwardC = Cvelocity * turnSpeed;
        double FeedBackX = PID(measuredVelocityX, "X");
        double FeedBackY = PID(measuredVelocityY, "Y");
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
}
