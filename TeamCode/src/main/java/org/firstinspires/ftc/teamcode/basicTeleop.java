package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Robot;

public class basicTeleop extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void autoOpenWitheStone(boolean reason){
        boolean [] upStep;
        upStep = new boolean[5];
        upStep[1] = true;

        if (reason) {
            if (upStep[1] == true){
                Output.setPosition(0.75);
                upStep[2] = true;
            }
            if (upStep[2] == true && Output.getPosition() > 0.6){
                MyElevator.moveElevator(1,1);
                upStep[3] = true;
                upStep[2] = false;
            }
            if (upStep[3] = true && upMagnetElevator.getState() == false){
                MyElevator.moveElevator(0,0);
                Arm.setPosition(0.104020);
                upStep[4] = true;
                upStep[3] = false;
            }
            if (upStep[4] == true && Arm.getPosition() > 0.09){
                MyElevator.moveElevator(1,1);
            }
        }
    }
}
