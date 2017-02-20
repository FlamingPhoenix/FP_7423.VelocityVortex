package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import FlamingPhoenix.MecanumDriveTrain;

/**
 * Created by HwaA18 on 11/5/2016.
 */

@TeleOp (name = "Warner", group = "Teleop")
@Disabled
public class WarnerBot extends OpMode {
    MecanumDriveTrain DriveTrain;
    ModernRoboticsI2cGyro gyro;

    @Override
    public void init()  {
        DriveTrain = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this); //Initialize a drive train using MecanumDriveTrain
    }

    @Override
    public void loop() {
        DriveTrain.Drive(this.gamepad1, 0.5f);
    }
}

