package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class elevator  {

    private DcMotor leftEle = null;
    private DcMotor rightEle = null;

    public elevator (DcMotor leftEle, DcMotor rightEle){
        this.leftEle = leftEle;
        this.rightEle = rightEle;
    }

    public void setPower(double leftPower, double rightPower){
        leftEle.setPower(leftPower);
        rightEle.setPower(rightPower);
    }

}
