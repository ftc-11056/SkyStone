package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

public class CustomadeIntake extends Robot {

    //    DCmotors:
    public DcMotor IntakeL = null;
    public DcMotor IntakeR = null;

    public double cubeNotInMM = 150;


    public CustomadeIntake(DcMotor IntakeL, DcMotor IntakeR) {
        this.IntakeL = IntakeL;
        this.IntakeR = IntakeR;

    }

    public void move(double IL, double IR) {
        IntakeL.setPower(IL);
        IntakeR.setPower(IR);
    }

    public void maxOuttake() {
        IntakeL.setPower(1);
        IntakeR.setPower(1);
    }

    public void maxIntake() {
        IntakeL.setPower(-1);
        IntakeR.setPower(-1);
    }

    public void ShutDown() {
        IntakeL.setPower(0);
        IntakeR.setPower(0);

    }

    public void LowOuttake(){
        IntakeL.setPower(0.5);
        IntakeR.setPower(0.5);
    }


}
