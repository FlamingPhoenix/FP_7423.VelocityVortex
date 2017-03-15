package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import FlamingPhoenix.Direction;
import FlamingPhoenix.MecanumDriveTrain;
import FlamingPhoenix.MyUtility;
import FlamingPhoenix.TurnDirection;

/**
 * Created by brandon on 12/29/2016.
 */

// @Disabled //
@Autonomous(name = "RedAuto-2", group = "")
public class RedFar extends LinearOpMode {
    ModernRoboticsI2cGyro gyro;

    private VuforiaLocalizer vuforia;
    VuforiaTrackables tracker;

    DcMotor shooter;
    DcMotor collecter;

    Servo stopper;

    ColorSensor color;

    Servo pusher;

    @Override
    public void runOpMode() throws InterruptedException {
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        gyro.resetZAxisIntegrator();

        color = hardwareMap.colorSensor.get("color");
        color.enableLed(false);

        pusher = hardwareMap.servo.get("pusher");

        collecter = hardwareMap.dcMotor.get("collector");

        pusher.setPosition(.5);

        while (gyro.isCalibrating() && this.opModeIsActive())
            Thread.sleep(50);

        MecanumDriveTrain wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", "leftwheels", "rightwheels", this);

        shooter = hardwareMap.dcMotor.get("farriswheel");
        stopper = hardwareMap.servo.get("stopper");

        stopper.setPosition(.75);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.idle();
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        gyro.resetZAxisIntegrator();
        gyro.calibrate();

        while(gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        waitForStart();

        wheels.drive(2, Direction.FORWARD, 0.4, 5, this);

        this.sleep(500);

        wheels.turnWithGyro(52, .25, TurnDirection.RIGHT, gyro, this);

        this.sleep(500);

        wheels.strafe(18, .5, TurnDirection.LEFT, this);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.idle();
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.idle();
        this.idle();

        shooter.setMaxSpeed(2550);

        shooter.setPower(1);

        this.sleep(3000);

        stopper.setPosition(.20);
        Thread.sleep(250);
        stopper.setPosition(.75);
        Thread.sleep(1050);
        stopper.setPosition(0.20);
        Thread.sleep(500);
        stopper.setPosition(.75);

        Thread.sleep(300);

        shooter.setPower(0);

        Thread.sleep(3000);

        int heading=gyro.getIntegratedZValue();

        wheels.turnWithGyro(Math.abs(-92-heading), .25, TurnDirection.RIGHT, gyro, this);

        Thread.sleep(4000);

        wheels.drive(95, Direction.BACKWARD, 0.75, 10, this);
    } 

}


