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
        leftEle.setPower(leftPower);
        rightEle.setPower(rightPower);
    }
    public boolean stateUpMagnet(){
        return upMagnet.getState();
    }
    public boolean stateDownMagnet(){
        return downMagnet.getState();
    }

}

