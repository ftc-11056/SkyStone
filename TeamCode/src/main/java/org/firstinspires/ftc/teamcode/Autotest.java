/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="PurePursuit", group="Auto")
@Disabled
public class Autotest extends OpMode{

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    private PathBuilder MyPathBuilder;
    private PurePursuitGUI MyPurePursuitGUI;
    private Odometry MyOdometry;
    private DriveTrain MyDriveTrain = new DriveTrain();
    private BNO055IMU imu = MyDriveTrain.getImu() ;
    private Orientation angles;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        MyDriveTrain.init(hardwareMap);
        OurPoint StartRobotPosition = new OurPoint(0,0);
        double startRobotDirection = 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);;
        MyOdometry = new Odometry(StartRobotPosition, startRobotDirection);


        double MaxVelocity = 1.727875947;
        double MaxAcceleration = 2;
        double turnSpeed = 0.75;
        double targetDirection = 0;
        double Kc = 2;
        double lookAheadDistance = 0.2;
        double Kv = 1/ MaxVelocity;
        double Ka = 0;
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        telemetry.addLine("init1");
        MyPathBuilder = new PathBuilder(MaxVelocity, MaxAcceleration, Kc);
        telemetry.addLine("init2");
        MyPurePursuitGUI = new PurePursuitGUI(StartRobotPosition, startRobotDirection, targetDirection, lookAheadDistance, turnSpeed, MaxAcceleration, Kv, Ka, Kp, Ki, Kd, MyOdometry);
        telemetry.addLine("init3");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double currentTime = runtime.seconds();
        double odometryRight = MyDriveTrain.RF.getCurrentPosition();
        double odometryLeft = MyDriveTrain.LF.getCurrentPosition();
        double odometryHorizental = MyDriveTrain.RB.getCurrentPosition();
        double direction = MyDriveTrain.imu.getAngularOrientation();
        MyOdometry.setAll(odometryRight, odometryLeft, odometryHorizental, direction, currentTime);
        MyPurePursuitGUI.UpdatePowerByRobotPosition(runtime.seconds(), MyOdometry.getPosition(), MyOdometry.getDirection(), MyOdometry.getVelocityX(), MyOdometry.getVelocityY());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }


}
