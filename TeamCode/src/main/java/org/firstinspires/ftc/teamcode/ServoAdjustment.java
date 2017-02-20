package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import FlamingPhoenix.MecanumDriveTrain;

/**
 * Created by HwaA1 on 12/10/2016.
 */

@TeleOp (name = "PusherAdjustment", group = "none")
public class ServoAdjustment extends OpMode {

    Servo pusher;
    Servo stopper;

    @Override
    public void init() {
        pusher = hardwareMap.servo.get("pusher");
        stopper = hardwareMap.servo.get("stopper");
    }

    @Override
    public void loop() {

        stopper.setPosition(.75);

        if(gamepad1.a) {
            pusher.setPosition(1);
        }
        else if(gamepad1.y) {
            pusher.setPosition(0);
        }
        else {
            pusher.setPosition(.5);
        }
    }
}
