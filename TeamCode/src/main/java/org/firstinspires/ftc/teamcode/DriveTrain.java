package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {

    /* Fileds */

    public DcMotor LeftBack = null;
    public DcMotor  LeftForward = null;
    public DcMotor  RightForward = null;
    public DcMotor  RightBack = null;

    /* Constructor */
     public DriveTrain(DcMotor LB, DcMotor  LF, DcMotor  RF, DcMotor  RB){
         this.LeftBack = LB;
         this.LeftForward = LF;
         this.RightForward = RF;
         this.RightBack = RB;
     }

    /* Methodes */



}
