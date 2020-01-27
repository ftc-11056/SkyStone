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
        IntakeL.setPower(0.6);
        IntakeR.setPower(0.6);
    }

    public void maxIntake() {
        IntakeL.setPower(-0.8);
        IntakeR.setPower(-0.8);
    }

    public void ShutDown() {
        IntakeL.setPower(0);
        IntakeR.setPower(0);

    }


}
