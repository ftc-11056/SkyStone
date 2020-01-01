package org.firstinspires.ftc.teamcode.PP;


import org.firstinspires.ftc.teamcode.PP.OurPoint;

public class PathFollowerTank {

    //fields:

    private OurPoint RobotPosition;
    private double robotDirection;
    private Object[][] wayPoint;
    private int lustClosetPointIndex;
    private final double lookAheadDistance;
    private final double trackWidth;
    private double MaxAcceleration;
    private double lustOutputRateLimiter;
    private double lustTimeRateLimiter;
    private double rightTargetVelocity;
    private double leftTargetVelocity;
    private double lastRightTargetVelocity;
    private double lastLeftTargetVelocity;
    private double rightTargetAcceleration;
    private double leftTargetAcceleration;
    private final double Kv;
    private final double Ka;
    private final double Kp;
    private final double Ki;
    private final double Kd;
    private double lastVelociityErrorRight;
    private double lastVelociityErrorLeft;
    private double integralRight;
    private double integralLeft;
    private double rightPower;
    private double leftPower;

    //constractors:

    public PathFollowerTank(OurPoint RobotPosition, double robotDirection, Object[][] wayPoint, double lookAheadDistance, double trackWidth, double MaxAcceleration, double Kv, double Ka, double Kp, double Ki, double Kd){
        this.RobotPosition = RobotPosition;
        this.robotDirection = robotDirection;
        this.wayPoint = wayPoint;
        lustClosetPointIndex = 0;
        this.lookAheadDistance = lookAheadDistance;
        this.trackWidth = trackWidth;
        this.MaxAcceleration = MaxAcceleration;
        this.lustOutputRateLimiter = 0;
        lustTimeRateLimiter = 0;
        rightTargetVelocity = 0;
        leftTargetVelocity = 0;
        lastRightTargetVelocity = 0;
        lastLeftTargetVelocity = 0;
        rightTargetAcceleration = 0;
        leftTargetAcceleration = 0;
        this.Kv = Kv;
        this.Ka = Ka;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        lastVelociityErrorRight = 0;
        lastVelociityErrorLeft = 0;
        integralRight = 0;
        integralLeft = 0;
        rightPower = 0;
        leftPower = 0;

    }

    //methodes:

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

    private double calculateCurvature(){
        OurPoint LookaheadPoint = findLookaheadPoint();
        double a = -Math.tan(robotDirection);
        double b = 1;
        double c = Math.tan(robotDirection)*RobotPosition.getX()- RobotPosition.getY();
        double x = MyMath.distanceBetweenLineAndPoint(a, b, c, LookaheadPoint);
        double curvature = (2*x)/MyMath.square(lookAheadDistance);
        curvature *= side(LookaheadPoint);
        return curvature;
    }

    private double side(OurPoint LookaheadPoint){
        OurPoint PointOnRobotLine = new OurPoint(RobotPosition.getX() + Math.cos(robotDirection), RobotPosition.getY() + Math.sin(robotDirection));
        double crossProduct = MyMath.CrossProduct(RobotPosition, PointOnRobotLine, RobotPosition, LookaheadPoint);
        if(crossProduct >= 0 )
            return 1;
        else
            return -1;
    }

    private void UpdateVelocitiesByRobotPosition(double time, OurPoint RobotPosition, double robotDirection){
        this.RobotPosition = RobotPosition;
        this.robotDirection = robotDirection;
        int closetPointIndex = findClosetPointIndex();
        double curvature = calculateCurvature();
        double targetVelocity = (Double)wayPoint[closetPointIndex][1];
        targetVelocity = rateLimiter(targetVelocity, time);
        rightTargetVelocity = (targetVelocity*(2+curvature*trackWidth))/2;
        leftTargetVelocity = (targetVelocity*(2-curvature*trackWidth))/2;
        rightTargetAcceleration = (rightTargetVelocity - lastRightTargetVelocity) / time;
        leftTargetAcceleration = (leftTargetVelocity - lastLeftTargetVelocity) / time;
        lastRightTargetVelocity = rightTargetVelocity;
        lastLeftTargetVelocity = leftTargetVelocity;
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

    private double PID(double measuredVelocity, String Side){
        double PID = 0;
        if(Side.equals("Right")){
            double error = lastRightTargetVelocity - measuredVelocity;
            integralRight += error;
            double derivative = error - lastVelociityErrorRight;
            PID = Kp*error + Ki*integralRight + Kd*derivative;
            lastVelociityErrorRight = error;
        }
        if(Side.equals("Left")){
            double error = lastLeftTargetVelocity - measuredVelocity;
            integralLeft += error;
            double derivative = error - lastVelociityErrorLeft;
            PID = Kp*error + Ki*integralLeft + Kd*derivative;
            lastVelociityErrorLeft = error;
        }
        return PID;
    }

    public void UpdatePowerByRobotPosition(double time, OurPoint RobotPosition, double robotDirection, double measuredVelocityRight, double measuredVelocityLeft){
        UpdateVelocitiesByRobotPosition(time, RobotPosition, robotDirection);
        double FeedForwardRight = Kv*rightTargetVelocity + Ka*rightTargetAcceleration;
        double FeedForwardLeft = Kv*leftTargetVelocity + Ka*leftTargetAcceleration;
        double FeedBackRight = PID(measuredVelocityRight, "Right");
        double FeedBackLeft = PID(measuredVelocityLeft, "Left");
        rightPower = FeedForwardRight + FeedBackRight;
        leftPower = FeedForwardLeft + FeedBackLeft;
    }

    public double getRightPower() {
        return rightPower;
    }

    public double getLeftPower() {
        return leftPower;
    }

}
