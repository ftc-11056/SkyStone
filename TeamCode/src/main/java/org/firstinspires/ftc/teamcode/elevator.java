package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class elevator  {

    private DcMotor leftEle = null;
    private DcMotor rightEle = null;
    private DigitalChannel upMagnet = null;
    private DigitalChannel downMagnet = null;
    public int fixedPosition = 0;


    public elevator (DcMotor leftEle, DcMotor rightEle, DigitalChannel upMagnet, DigitalChannel downMagnet, int fixedPosition){
        this.leftEle = leftEle;
        this.rightEle = rightEle;
        this.upMagnet = upMagnet;
        this.downMagnet = downMagnet;
//        this.fixedPosition = fixedPosition;

        this.leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveElevator (double leftPower, double rightPower){
        leftEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEle.setPower(leftPower);
        rightEle.setPower(rightPower);
    }

    public void ElevateWithEncoder(int pos, double power){

        int newLeftTargetPositin = leftEle.getCurrentPosition() + pos;
        int newrightTargetPositin = rightEle.getCurrentPosition() + pos;
        leftEle.setTargetPosition(newLeftTargetPositin);
        rightEle.setTargetPosition(newrightTargetPositin);
        leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftEle.setPower(Math.abs(power));
        rightEle.setPower(Math.abs(power));
        fixedPosition = leftEle.getCurrentPosition();
    }

    public void dontMoveElevator (double power, double PN,int leftCurrent, int rightCurrent){
        double error = fixedPosition - leftEle.getCurrentPosition();
        leftEle.setTargetPosition(leftCurrent);
        rightEle.setTargetPosition(rightCurrent);
        leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftEle.setPower(power /*+ error * PN*/);
        rightEle.setPower(power /*+ error * PN*/);
    }

    public boolean stateUpMagnet(){
        return upMagnet.getState();
    }
    public boolean stateDownMagnet(){
        return downMagnet.getState();
    }
    public Boolean isBusy (){
        if (leftEle.isBusy() && rightEle.isBusy())
            return true;
        else return false;
    }


}

