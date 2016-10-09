package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import FlamingPhoenix.*;

/**
 * Created by Steve on 9/24/2016.
 */

@TeleOp(name = "Mechanum", group = "Practice")
public class ProgrammingBot extends OpMode {

    MecanumDriveTrain DriveTrain;

    @Override
    public void init() {
        DriveTrain = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this); //Initialize a drive train using MecanumDriveTrain
    }

    @Override
    public void loop() {
        DriveTrain.Drive(this.gamepad1);
    }
}
