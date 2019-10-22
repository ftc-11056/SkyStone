package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeTrain extends Robot {

//    DCmotors:
    public DcMotor IntakeL = null;
    public DcMotor IntakeR = null;

    public IntakeTrain(DcMotor IntakeL, DcMotor IntakeR){
        this.IntakeL = IntakeL;
        this.IntakeR = IntakeR;

    }

    public void move (double IL, double IR) {
        IntakeL.setPower(IL);
        IntakeR.setPower(IR);
    }

    public void maxIntake(){
        IntakeL.setPower(1);
        IntakeR.setPower(1);
    }

    public void maxOuttake(){
        IntakeL.setPower(-1);
        IntakeR.setPower(-1);
    }
    public void ShutDown(){
        IntakeL.setPower(0);
        IntakeR.setPower(0);

    }
}
