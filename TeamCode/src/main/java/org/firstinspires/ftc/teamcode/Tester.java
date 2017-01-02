package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import FlamingPhoenix.MecanumDriveTrain;
import FlamingPhoenix.MyMotor;

/**
 * Created by Steve on 1/1/2017.
 */

@Autonomous(name = "Tester", group = "none")
@Disabled()
public class Tester extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    VuforiaTrackables tracker;

    MyMotor shooter;
    Servo stopper;
    ColorSensor color;
    OpticalDistanceSensor opt;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        shooter = new MyMotor(hardwareMap.dcMotor.get("farriswheel"));
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.idle();
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMaxSpeed(900);

        waitForStart();

        shooter.setPower(1);
        this.telemetry.addData("reached speed", "curentPower=%8.5f", shooter.getPower());
        this.telemetry.update();

        stopper.setPosition(.25);
        Thread.sleep(250);
        stopper.setPosition(.75);
        Thread.sleep(500);
        stopper.setPosition(0.25);
        Thread.sleep(1000);
        stopper.setPosition(.75);
        shooter.setPower(0);

        Thread.sleep(2000);
    }

    //Initialize to set up robot
    private void initialize() throws InterruptedException {
        stopper = hardwareMap.servo.get("stopper");
        stopper.setPosition(.75);

        color = hardwareMap.colorSensor.get("color");
        color.enableLed(false);

        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parms);

        tracker = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        tracker.activate();

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        gyro.resetZAxisIntegrator();

        opt = hardwareMap.opticalDistanceSensor.get("opt");

        while (gyro.isCalibrating() && this.opModeIsActive())
            Thread.sleep(50);

        MecanumDriveTrain wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);


        gyro.resetZAxisIntegrator();
        gyro.calibrate();

        while(gyro.isCalibrating()) {
            Thread.sleep(50);
        }
    }
}
