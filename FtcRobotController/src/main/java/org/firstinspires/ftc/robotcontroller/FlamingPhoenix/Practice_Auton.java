package org.firstinspires.ftc.robotcontroller.FlamingPhoenix;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by brand on 9/29/2016.
 */

public class Practice_Auton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveTrain wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);

        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro");

        waitForStart();

        wheels.Drive(12,25,this);
        wheels.turnUsingGyro(90, 40, TurnDirection.RIGHT, false, gyro, this);
    }
}
