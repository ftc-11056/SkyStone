package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

    /* Public OpMode members. */
    public DcMotor  LB   = null;
    public DcMotor  LF  = null;
    public DcMotor  RF     = null;
    public DcMotor  RB     = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public DriveTrain(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LF  = hwMap.get(DcMotor.class, "LF");
        LB  = hwMap.get(DcMotor.class, "LB");
        RB  = hwMap.get(DcMotor.class, "RB");
        RF  = hwMap.get(DcMotor.class, "RF");


        LF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        LB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.

    }

}
