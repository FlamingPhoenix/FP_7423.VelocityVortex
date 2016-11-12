package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import FlamingPhoenix.*;

/**
 * Created by brand on 9/29/2016.
 */

@Autonomous(name = "Auton", group = "Practice")
public class Practice_Auton extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        while (gyro.isCalibrating())
            Thread.sleep(50);

        MecanumDriveTrain wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);

        //gyro.calibrate();

        waitForStart();

        wheels.Drive(28, .3, this);
        wheels.strafe(15, .5, TurnDirection.LEFT, gyro, this);
        wheels.Drive(32, .3, this);
    }
}
