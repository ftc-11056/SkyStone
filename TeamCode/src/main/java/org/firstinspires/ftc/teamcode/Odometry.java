package org.firstinspires.ftc.teamcode;


public class Odometry {

    //fields:

    private OurPoint Position;
    private double direction;
    private double lastIncoderRight;
    private double lastIncoderLeft;
    private double lastIncoderSide;
    private double velocityX;
    private double velocityY;
    private double lastTime;
    private final double wheelCicurmference = 0.159592772;
    private final double ticForRound = 600;
    private final double convertIncoderToM = wheelCicurmference / ticForRound;   //change this by encoder


    //constractors:

    public Odometry(OurPoint RobotPosition,double direction){
        Position = RobotPosition;
        this.direction = direction;
        lastIncoderRight = 0;
        lastIncoderLeft = 0;
        lastIncoderSide = 0;
        velocityX = 0;
        velocityY = 0;
        lastTime = 0;
    }

    public Odometry(){
        Position = new OurPoint();
        this.direction = 0;
        lastIncoderRight = 0;
        lastIncoderLeft = 0;
        lastIncoderSide = 0;
    }


    //methodes:

    public OurPoint getPosition() {
        return Position;
    }

    public double getDirection() {
        return direction;
    }

    public void setAll(double currentIncoderRight ,double currentIncoderLeft ,double currentIncoderSide ,double direction, double time) {
        setDirection(direction);
        double sinD = Math.sin(direction);
        double cosD = Math.sin(direction);

        currentIncoderRight *= convertIncoderToM;
        currentIncoderLeft *= convertIncoderToM;
        currentIncoderSide *= convertIncoderToM;

        double dTime = time - lastTime;
        double dRight = currentIncoderRight - lastIncoderRight;
        double dLeft = currentIncoderLeft - lastIncoderLeft;
        double dForward = (dRight + dLeft)/2;
        double dSide = currentIncoderSide - lastIncoderSide;

        double dX = dForward*cosD + dSide*cosD;
        double dY = dForward*sinD + dSide*sinD;

        velocityX = dX / dTime;
        velocityY = dY / dTime;

        movePosition(dX, dY);

        lastIncoderRight = currentIncoderRight;
        lastIncoderLeft = currentIncoderLeft;
        lastIncoderSide = currentIncoderSide;
    }

    private void movePosition(double dX, double dY) {
        Position.move(dX,dY);
    }

    private void setDirection(double direction) {
        this.direction = direction;
    }

    public double getVelocityX() {
        return velocityX;
    }

    public double getVelocityY() {
        return velocityY;
    }
}

