package org.firstinspires.ftc.teamcode.PP;

public class OdometryCustomMade {

    //fields:

    private OurPoint Position;
    private double lastEncoderRight;
    private double lastEncoderLeft;
    private double lastEncoderSide;
    private double velocityX;
    private double velocityY;
    private double lastVelocityX;
    private double lastVelocityY;
    private double AccelerationX;
    private double AccelerationY;
    private double lastTime;


    //Custom made Robot measurements:
    private final double distanceVerticalToCenter = 0.08379;
    public final double distanceHorizentalToCenter = 0.09594;
    private final double wheelCicurmference = 0.159592772;
    private final double ticForRound = 8192;
    private final double ratio = 1;



    private final double convertEncoderToM = wheelCicurmference / (ticForRound * ratio);

    public double dSide = 0;
    public double dForward = 0;
    public double dX = 0;
    public double dY = 0;
    public double dRight = 0;
    public double dLeft = 0;
    public double dCenter = 0;
    public double dAngle = 0;
    public double currentAngle = 0;
    public double dTime = 0;

    //debug parameters:
    public double xRobot = 0;
    public double yRobot = 0;
    public double DeltaAngle = 0;
    public double RminusL = 0;

    //constractors:

    public OdometryCustomMade(OurPoint RobotPosition){
        Position = RobotPosition;
        Position.setRadAngle(Position.getRadAngle() + (Math.PI / 2));
        lastEncoderRight = 0;
        lastEncoderLeft = 0;
        lastEncoderSide = 0;
        velocityX = 0;
        velocityY = 0;
        lastVelocityX = 0;
        lastVelocityY = 0;
        AccelerationX = 0;
        AccelerationY = 0;
        lastTime = 0;
    }

    public OdometryCustomMade(){
        Position = new OurPoint();
        Position.setRadAngle(Position.getRadAngle() + (Math.PI / 2));
        lastEncoderRight = 0;
        lastEncoderLeft = 0;
        lastEncoderSide = 0;
    }


    //methodes:

    public OurPoint getPosition() {
        return Position;
    }

    public double getDirection() {
        return Position.getRadAngle();
    }

    public void setAll(double currentEncoderRight ,double currentEncoderLeft ,double currentEncoderSide , double time) {
        currentEncoderRight *= convertEncoderToM;
        currentEncoderLeft *= convertEncoderToM;
        currentEncoderSide *= convertEncoderToM;

        dTime = time - lastTime;
        dRight = currentEncoderRight - lastEncoderRight;
        dLeft = currentEncoderLeft - lastEncoderLeft;
        dCenter = currentEncoderSide - lastEncoderSide;

        dAngle = (dRight - dLeft) / (2*distanceVerticalToCenter);
        currentAngle = dAngle;
        dForward = (dRight + dLeft)/2;
        dSide = (distanceHorizentalToCenter*(dLeft - dRight))/(2*distanceVerticalToCenter) + dCenter;
        Position.setRadAngle(Position.getRadAngle() + dAngle);

        double sinD = Math.sin(Position.getRadAngle());
        double cosD = Math.cos(Position.getRadAngle());

        dX = dForward*cosD + dSide*sinD;
        dY = dForward*sinD -(dSide*cosD);

        //Debug:
        xRobot += dSide;
        yRobot += dForward;
        DeltaAngle +=dAngle;
        RminusL +=(dRight - dLeft);
        updateVelocities();
        updateAcceleration();


        Position.move(dX,dY);

        lastEncoderRight = currentEncoderRight;
        lastEncoderLeft = currentEncoderLeft;
        lastEncoderSide = currentEncoderSide;
        lastTime = time;
        lastVelocityX = velocityX;
        lastVelocityY = velocityY;
    }

    public void decreaseAngle(){
        double NewAngle = MyMath.AngleMap(Position.getRadAngle());
        Position.setRadAngle(NewAngle);

    }

    public void updateAcceleration(){
        if(dTime <=0.025){
            return;
        }
        double dVelocityX = velocityX - lastVelocityX;
        double dVelocityY = velocityY - lastVelocityY;
        double currAccelerationX = dVelocityX / dTime;
        double currAccelerationY = dVelocityY / dTime;
        if(dVelocityX >= 0.000000001 && currAccelerationX < 200){
            AccelerationX = currAccelerationX;
        }
        if(dVelocityY >= 0.000000001 && currAccelerationY < 200){
            AccelerationY = currAccelerationY;
        }
    }

    public void updateVelocities(){
        if(dTime <=0.025){
            return;
        }
        double currVelocityX = dSide / dTime;
        double currVelocityY = dForward / dTime;
        if(Math.abs(dSide) >= 0.000000001 && currVelocityX < 200){
            velocityX = currVelocityX;
        }
        if(Math.abs(dForward) >=0.000000001 && currVelocityY <= 200){
            velocityY = currVelocityY;
        }
    }

    public double getVelocityX() {
        return velocityX;
    }

    public double getVelocityY() {
        return velocityY;
    }

    public double getVelocity(){
        return Math.sqrt(MyMath.square(velocityX) + MyMath.square(velocityY));
    }

    public double getAccelerationX() {
        return AccelerationX;
    }

    public double getAccelerationY() {
        return AccelerationY;
    }
}

