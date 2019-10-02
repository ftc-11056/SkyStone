package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="LoadingZone", group="teamcode")
public class LoadingZone extends VuforiaPhotoID{

    VuforiaSkyStone Web = new VuforiaSkyStone();



    @Override
    public void runOpMode() {
        super.runOpMode();

       // Web.getImageLocation(translitionY);
        waitForStart();

        switch (LocationY){
            case ("Right"):
                telemetry.addLine("Meitar");
                break;
            case ("Center"):
                telemetry.addLine("Menachem");
                break;
            case ("Left"):
                telemetry.addLine("Clil");
                break;
                default:
                    telemetry.addLine("Ori");
                    break;
            }
            telemetry.update();
        sleep(11056);



        }
    }



