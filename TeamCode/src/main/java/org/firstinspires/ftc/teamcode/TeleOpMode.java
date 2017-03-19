package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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
    Servo pusher;
    Servo poker;

    boolean isInPosition = false;

    double Onoroff;
    int counter;
    boolean stop;

    int previousCount;
    long prevTime;


    @Override
    public void init() {
        try {
            DriveTrain = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);
        } catch (InterruptedException e) {}

        shooter = hardwareMap.dcMotor.get("farriswheel");
        collector = hardwareMap.dcMotor.get("collector");
        poker = hardwareMap.servo.get("poker");

        pusher = hardwareMap.servo.get("pusher");
        pusher.setPosition(.5);
        poker.setPosition(0.55);

        //stopper = hardwareMap.servo.get("");

        stopper = hardwareMap.servo.get("stopper");

        stopper.setPosition(.75);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMaxSpeed(2625);
        Onoroff = 0;

        counter = 0;

        previousCount = shooter.getCurrentPosition();
        prevTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        if(gamepad2.right_bumper && gamepad2.left_bumper) {
            Onoroff = .1;
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            stop = true;
        }
        else if(gamepad2.y) {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Onoroff = .1;
            counter = 0;
            stop = false;

        }
        else if((counter > 10) && (Onoroff != 0)) { //10 loops is approximately 1 seconds
            if(stop)
                Onoroff = 0;
            else
                Onoroff = 1;
        }

        shooter.setPower(Onoroff);

        if(gamepad2.right_bumper) {
            collector.setPower(1);
        }
        else if(gamepad2.left_bumper)
            collector.setPower(-1);
        else
            collector.setPower(0);

        if(gamepad1.a) {
            pusher.setPosition(1);
        }
        else if(gamepad1.y) {
            pusher.setPosition(0);
        }
        else {
            pusher.setPosition(.5);
        }


        DriveTrain.Drive(gamepad1);

        if((gamepad2.right_trigger > .2) && !(gamepad2.right_bumper) && !stop) {
            stopper.setPosition(.25);
            this.resetStartTime();
            isInPosition = true;
        }

        if(this.time > 0.2 && isInPosition) {
            stopper.setPosition(.75);
            isInPosition = false;
        }

        counter++;

        if(counter >= 100)
            counter = 100;

        long currentTime = System.currentTimeMillis();
        long timeDifference = currentTime - prevTime;
        double speed = 0;
        int newCount = shooter.getCurrentPosition();
        if(timeDifference > 1000) {
            speed = ((newCount - previousCount) * 1000 / timeDifference);


            previousCount = newCount;
            prevTime = currentTime;

            this.telemetry.addData("speed", speed);
            this.telemetry.update();
        }

    }
}
