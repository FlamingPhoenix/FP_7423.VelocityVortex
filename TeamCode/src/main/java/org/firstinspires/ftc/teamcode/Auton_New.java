package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import FlamingPhoenix.MyUtility;
/**
 * Created by HwaA1 on 11/26/2016.
 */
@Autonomous(name = "Turn_Test", group = "none")
public class Auton_New extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    VuforiaTrackables tracker;

    DcMotor shooter;

    Servo stopper;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parms);

        tracker = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        tracker.activate();

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        gyro.resetZAxisIntegrator();

        while (gyro.isCalibrating() && this.opModeIsActive())
            Thread.sleep(50);

        MecanumDriveTrain wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);

        shooter = hardwareMap.dcMotor.get("farriswheel");
        stopper = hardwareMap.servo.get("stopper");

        stopper.setPosition(.75);

        shooter.setMaxSpeed(2400);

        gyro.resetZAxisIntegrator();
        gyro.calibrate();

        while(gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        waitForStart();

        //shooter.setPower(.3);
        Thread.sleep(1000);
        //shooter.setPower(1);

        wheels.strafe(8, 1200, TurnDirection.LEFT, gyro,this);
        //stopper.setPosition(.25);
        Thread.sleep(2000);
        //stopper.setPosition(.75);

        Thread.sleep(1000);

        //shooter.setPower(.3);

        wheels.strafe(10, 1200, TurnDirection.LEFT, gyro, this);

        shooter.setPower(0);

        wheels.drive(25, Direction.BACKWARD, 1400, this);
        wheels.turnWithGyro(90, .45, TurnDirection.LEFT, gyro, this);
        wheels.drive(30, Direction.FORWARD, 1400, this);

        Thread.sleep(250);
        int angle = MyUtility.getImageAngle(tracker.get(3));

        DbgLog.msg("[Phoenix] angle: " + angle);

        int turnAngle = Math.abs(90 - angle);

        if(angle < 90)
            wheels.turnWithGyro(turnAngle, .5, TurnDirection.RIGHT, gyro, this);
        else if(angle > 90)
            wheels.turnWithGyro(turnAngle, .5, TurnDirection.LEFT, gyro, this);
        else {}

        wheels.strafe(200, 1400, TurnDirection.LEFT, tracker.get(3), gyro, this);
    }

}
