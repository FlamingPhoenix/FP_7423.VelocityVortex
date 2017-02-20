package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import FlamingPhoenix.TurnDirection;

/**
 * Created by brandon on 1/26/2017.
 */

// @Disabled //
@Autonomous(name = "BlueAuto-2", group = "")
public class BlueFar extends LinearOpMode{

    ModernRoboticsI2cGyro gyro;

    private VuforiaLocalizer vuforia;
    VuforiaTrackables tracker;

    DcMotor shooter;

    Servo stopper;

    ColorSensor color;

    @Override
    public void runOpMode() throws InterruptedException {
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        gyro.resetZAxisIntegrator();

        color = hardwareMap.colorSensor.get("color");
        color.enableLed(true);

        while (gyro.isCalibrating() && this.opModeIsActive())
            Thread.sleep(50);

        MecanumDriveTrain wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);

        shooter = hardwareMap.dcMotor.get("farriswheel");
        stopper = hardwareMap.servo.get("stopper");

        stopper.setPosition(.75);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.idle();
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMaxSpeed(960);

        gyro.resetZAxisIntegrator();
        gyro.calibrate();

        while(gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        waitForStart();

        Thread.sleep(10000);

        wheels.drive(5, Direction.FORWARD, 0.4, 5, this);

        wheels.turnWithGyro(140, .3, TurnDirection.RIGHT, gyro, this);

        shooter.setPower(1);

        Thread.sleep(500);

        stopper.setPosition(.20);
        Thread.sleep(250);
        stopper.setPosition(.75);
        Thread.sleep(750);
        stopper.setPosition(0.20);
        Thread.sleep(500);
        stopper.setPosition(.75);

        Thread.sleep(300);

        shooter.setPower(0);

        wheels.turnWithGyro(47, .3, TurnDirection.LEFT, gyro, this);

        wheels.drive(83, Direction.FORWARD, 0.9, 10, this);
    }
}
