package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "TeleOpTest", group = "teamcode")
public class TeleOpTest extends Robot {

    private int grandFlag = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        while (opModeIsActive()) {

            angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = angles.firstAngle;

            if (gamepad1.x) MyDriveTrain.setMode("arcade");
            else if (gamepad1.b) MyDriveTrain.setMode("Oriented");

            telemetry.addData("Mode: ", MyDriveTrain.Mode);
            telemetry.update();

            if (MyDriveTrain.getMode().equals("Oriented")) {
                MyDriveTrain.fieldOriented(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, heading);
            } else {
                MyDriveTrain.arcade(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            /*if (gamepad1.left_bumper) {
                Arm.setPosition(0);
            } else if (gamepad1.right_bumper) {
                Arm.setPosition(1);
            }
            telemetry.addData("Arm Position", Arm.getPosition());
            telemetry.update();*/

           /* if (gamepad1.a) {
                Output.setPosition(1);

            } else if (gamepad1.y) {
                Output.setPosition(0);
            }*/

            if (gamepad1.dpad_left) {
                MyElevator.setPower(1, 1);
            } else if (gamepad1.dpad_right) {
                MyElevator.setPower(-1, -1);
            } else {
                MyElevator.setPower(0, 0);
            }

            if (gamepad1.right_bumper) {
                MyIntake.maxIntake();
            } else if (gamepad1.left_bumper) {
                MyIntake.maxOuttake();
            } else {
                MyIntake.move(0, 0);
            }
//////////////////////////////////////////////////////////////////////////////////////////////
            if (gamepad1.dpad_up){
                double flag = 0;
                grandFlag = 1;
                //סוגר על הקובייה
                if (flag == 0){
                    Output.setPosition(0.75);
                }
                //משנה את הזמן
                if (Output.getPosition() == 0.75) flag =1;
                //מרים את המעלית
                if (MyElevator.stateUpMagnet() && flag == 1){
                    MyElevator.setPower(1,1);
                    flag = 2;
                }
                else {
                    MyElevator.setPower(0,0);
                }
                //מסובב את הזרוע ב180 מעלות
                if (flag == 2){
                    Arm.setPosition(0.75);
                }

            }

            if (gamepad1.dpad_down) {
                double flag = 0;
                //מסובב בחזרה את הזרוע
                if (flag == 0){
                    Arm.setPosition(1);
                    Output.setPosition(1);
                }
                //משנה את הסמן
                if (Arm.getPosition() == 1 && Output.getPosition() == 1) flag =1;
                //מוריד את המעלית עד למעלה
                if (flag == 1 && MyElevator.stateDownMagnet()){
                    MyElevator.setPower(-0.5,-0.5);
                }
                if (downMagnetElevator.getState() == true) flag = 2;
            }
            /*if (gamepad1.right_trigger > 0) {
                LeftServo.setPosition(0.75);
                RightServo.setPosition(0.25);
            } else if (gamepad1.left_trigger > 0) {
                LeftServo.setPosition(0.4);
                RightServo.setPosition(0.5);
            }*/

        }
    }
}









