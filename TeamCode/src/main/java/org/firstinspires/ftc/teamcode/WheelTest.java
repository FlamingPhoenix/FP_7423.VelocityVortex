package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import FlamingPhoenix.MecanumDriveTrain;

/**
 * Created by HwaA1 on 12/10/2016.
 */

@TeleOp (name = "WheelTest", group = "none")
@Disabled
public class WheelTest extends OpMode {

    MecanumDriveTrain DriveTrain;

    DcMotor shooter;
    DcMotor collector;

    Servo stopper;

    boolean isInPosition = false;

    double Onoroff;
    int counter;
    boolean stop;
    int previousCount;
    long prevTime;

    @Override
    public void init() {

        shooter = hardwareMap.dcMotor.get("farriswheel");
        collector = hardwareMap.dcMotor.get("collector");

        //stopper = hardwareMap.servo.get("");

        stopper = hardwareMap.servo.get("stopper");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMaxSpeed(2650);
        Onoroff = 0;

        counter = 0;

        previousCount = shooter.getCurrentPosition();
        prevTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        shooter.setPower(1);

        int newCount = shooter.getCurrentPosition();
        long currentTime = System.currentTimeMillis();

        long timeDifference = currentTime - prevTime;

        double speed = 0;

        if(timeDifference > 1000) {
            speed = ((newCount - previousCount) * 1000 / timeDifference);

            previousCount = newCount;
            prevTime = currentTime;

            DbgLog.msg("[WheelTest] previous count: " + previousCount + ". newCount: " + newCount + ". speed: " + speed);

            this.telemetry.addData("speed", speed);
            this.telemetry.update();
        }
    }
}
