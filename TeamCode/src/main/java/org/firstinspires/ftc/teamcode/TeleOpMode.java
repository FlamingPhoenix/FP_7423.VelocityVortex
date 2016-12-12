package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import FlamingPhoenix.MecanumDriveTrain;

/**
 * Created by HwaA1 on 12/10/2016.
 */

@TeleOp (name = "TeleOp", group = "none")
public class TeleOpMode extends OpMode {

    MecanumDriveTrain DriveTrain;

    DcMotor shooter;
    DcMotor collector;

    Servo stopper;

    boolean isInPosition = false;

    @Override
    public void init() {
        //DriveTrain = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);

        //shooter = hardwareMap.dcMotor.get("farriswheel");
        //collector = hardwareMap.dcMotor.get("collector");

        //stopper = hardwareMap.servo.get("");

        stopper = hardwareMap.servo.get("stopper");

        stopper.setPosition(.75);

        //shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter.setMaxSpeed(1000);
    }

    @Override
    public void loop() {
        /*shooter.setPower(1);

        if(gamepad2.a) {
            collector.setPower(1);
        }*/

        if(gamepad2.right_trigger > .2) {
            stopper.setPosition(.25);
            this.resetStartTime();
            isInPosition = true;
        }

        if(this.time > 0.2 && isInPosition) {
            stopper.setPosition(.75);
            isInPosition = false;
        }

        this.telemetry.addData("time", this.time);
        this.telemetry.update();
    }
}
