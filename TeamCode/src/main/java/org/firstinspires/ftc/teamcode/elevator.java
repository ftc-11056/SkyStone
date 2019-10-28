package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class elevator  {

    private DcMotor leftEle = null;
    private DcMotor rightEle = null;
    private DigitalChannel upMagnet = null;
    private DigitalChannel downMagnet = null;


    public elevator (DcMotor leftEle, DcMotor rightEle, DigitalChannel upMagnet, DigitalChannel downMagnet){
        this.leftEle = leftEle;
        this.rightEle = rightEle;

        this.upMagnet = upMagnet;
        this.downMagnet = downMagnet;
    }

    public void setPower(double leftPower, double rightPower){
        leftEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEle.setPower(leftPower);
        rightEle.setPower(rightPower);
    }

    public void ElevateWithEncoder(int pos, double power){
        leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEle.setTargetPosition(pos);
        rightEle.setTargetPosition(pos);
        leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftEle.setPower(power);
        rightEle.setPower(power);

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

