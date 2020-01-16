package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class elevator {

    private DcMotor leftEle = null;
    private DcMotor rightEle = null;
    private DigitalChannel upMagnet = null;
    private DigitalChannel downMagnet = null;
    public int stay = 0;
    public double stayPower = 0;

    public elevator(DcMotor leftEle, DcMotor rightEle, DigitalChannel upMagnet, DigitalChannel downMagnet, int fixedPosition) {
        this.leftEle = leftEle;
        this.rightEle = rightEle;
        this.upMagnet = upMagnet;
        this.downMagnet = downMagnet;
//        this.fixedPosition = fixedPosition;

        this.leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveElevator(double leftPower, double rightPower) {
        leftEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEle.setPower(leftPower);
        rightEle.setPower(rightPower);
    }

    public void ElevateWithEncoder(int pos, double power, double kp) {
        leftEle.setTargetPosition(pos);
        rightEle.setTargetPosition(pos);
        leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftEle.setPower(power * (pos - leftEle.getCurrentPosition()) * kp);
        rightEle.setPower(power * (pos - leftEle.getCurrentPosition()) * kp);
    }

    public void setStayValues(int stay, double power){
        this.stay = stay;
        stayPower = power;
    }

    public int getStay(){
        return stay;
    }

    public double getStayPower(){
        return  stayPower;
    }

    public void ElevateCustomRight(int pos, double power, double kp) {
        leftEle.setTargetPosition(pos);
        leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (leftEle.getCurrentPosition() < -395) {
            rightEle.setPower(0);
            leftEle.setPower(0);
        }
        else  {
            rightEle.setPower(power * (pos - leftEle.getCurrentPosition()) * kp);
            leftEle.setPower(power * (pos - leftEle.getCurrentPosition()) * kp);
        }
    }

    public void ElevateWithEncoderTest(int pos, double power, double kp) {
        leftEle.setTargetPosition(pos);
        rightEle.setTargetPosition(pos);
        leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftEle.setPower(power * (pos - leftEle.getCurrentPosition()) * kp);
        rightEle.setPower(-(power * (pos - leftEle.getCurrentPosition()) * kp));
    }

    public void WithoutPElevateWithEncoder(int pos, double power) {
        leftEle.setTargetPosition(pos);
//        rightEle.setTargetPosition(pos);
        leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftEle.setPower(power);
        rightEle.setPower(power);
    }

    public void ElevateWithEncoderNew() {
        double power = 0;
        int newLeftTargetPositin = leftEle.getCurrentPosition() + 25;
        int newrightTargetPositin = rightEle.getCurrentPosition() + 25;
        leftEle.setTargetPosition(newLeftTargetPositin);
        rightEle.setTargetPosition(newrightTargetPositin);

        if ((leftEle.getCurrentPosition() < 100 && rightEle.getCurrentPosition() < 100) ||
                (leftEle.getCurrentPosition() > 450 && rightEle.getCurrentPosition() > 450))
            power = 0.1;
        else if (leftEle.getCurrentPosition() > 550 && rightEle.getCurrentPosition() > 550)
            power = 0;
        else power = 1;

        leftEle.setPower(Math.abs(power));
        rightEle.setPower(Math.abs(power));

        leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//        fixedPosition = leftEle.getCurrentPosition();
    }

    public void dontMoveElevator(double power, int stayingPosition/*,double PN,int leftCurrent, int rightCurrent*/) {
//        double error = fixedPosition - leftEle.getCurrentPosition();
        leftEle.setTargetPosition(stayingPosition);
        rightEle.setTargetPosition(stayingPosition);
        leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftEle.setPower(power /*+ error * PN*/);
        rightEle.setPower(power /*+ error * PN*/);
    }

    public boolean stateUpMagnet() {
        return upMagnet.getState();
    }

    public boolean stateDownMagnet() {
        return downMagnet.getState();
    }

    public Boolean isBusy() {
        if (leftEle.isBusy() && rightEle.isBusy())
            return true;
        else return false;
    }


}

