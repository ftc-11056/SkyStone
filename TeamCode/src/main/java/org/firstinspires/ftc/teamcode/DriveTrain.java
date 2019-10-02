package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

public class DriveTrain extends Robot {

    /* Fileds */

    public DcMotor LeftBack = null;
    public DcMotor LeftFront = null;
    public DcMotor RightFront = null;
    public DcMotor RightBack = null;


    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: ANDYMARK Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 10.16;     // For figuring circumference
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415926);

    public String Mode = "Oriented";

    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public DriveTrain(DcMotor LB, DcMotor LF, DcMotor RF, DcMotor RB) {
        this.LeftBack = LB;
        this.LeftFront = LF;
        this.RightFront = RF;
        this.RightBack = RB;
    }


    /* Methodes */

    public void arcade(double y, double x, double c) {
        double leftFrontVal = -y + x + c;
        double rightFrontVal = -y - x - c;
        double leftBackVal = -y - x + c;
        double rightBackVal = -y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        LeftFront.setPower(leftFrontVal);
        RightFront.setPower(rightFrontVal);
        RightBack.setPower(rightBackVal);
        LeftBack.setPower(leftBackVal);
    }

    public void fieldOriented(double y, double x, double c, double gyroheading) {
        double cosA = Math.cos(gyroheading);
        double sinA = Math.sin(gyroheading);
        double xOut = x * cosA - y * sinA;
        double yOut = x * sinA + y * cosA;
        arcade(yOut, xOut, c);
    }

    public String getMode() {
        return Mode;
    }

    public void setMode(String mode) {
        Mode = mode;
    }


    public void encoderDrive(double speed, double LeftFrontCM,double LeftBackCM, double RightFrontCM,double RightBackCM) {
        runtime.reset();
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;

        // Ensure that the opmode is still active


        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = LeftFront.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
        newLeftBackTarget = LeftBack.getCurrentPosition() + (int) (LeftBackCM * COUNTS_PER_CM);
        newRightFrontTarget = RightFront.getCurrentPosition() + (int) (RightFrontCM * COUNTS_PER_CM);
        newRightBackTarget = RightBack.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);

        LeftFront.setTargetPosition(newLeftFrontTarget);
        LeftBack.setTargetPosition(newLeftBackTarget);
        RightFront.setTargetPosition(newRightFrontTarget);
        RightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        /*runtime.reset();*/
        LeftFront.setPower(Math.abs(speed));
        LeftBack.setPower(Math.abs(speed));
        RightFront.setPower(Math.abs(speed));
        RightBack.setPower(Math.abs(speed));

        while ((LeftFront.isBusy() && LeftBack.isBusy() && RightFront.isBusy()) && RightBack.isBusy() && runtime.seconds() < Math.abs(LeftFrontCM / 35)) {

        }

        // Stop all motion;
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //  sleep(250);   // optional pause after each move
    }

    public void startAndResetEncoders () {

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}




