package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import FlamingPhoenix.Direction;
import FlamingPhoenix.MecanumDriveTrain;
import FlamingPhoenix.TurnDirection;

/**
 * Created by HwaA1 on 11/12/2016.
 */

@Autonomous(name = "Blue_Auton", group = "Autonomous")
public class Blue_Auton extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    VuforiaTrackables tracker;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parms);

        tracker = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        ColorSensor color = hardwareMap.colorSensor.get("color");
        OpticalDistanceSensor opt = hardwareMap.opticalDistanceSensor.get("opt");
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        while (gyro.isCalibrating() && this.opModeIsActive())
            Thread.sleep(50);

        MecanumDriveTrain wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);

        gyro.calibrate();

        tracker.get(0).setName("Wheels");
        tracker.get(1).setName("Tools");
        tracker.get(2).setName("Legos");
        tracker.get(3).setName("Gears");

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        tracker.activate();

        waitForStart();

        wheels.drive(28, Direction.BACKWARD, 1500, this);
        wheels.strafe(15, 1700, TurnDirection.LEFT, gyro, this);
        wheels.drive(25, Direction.BACKWARD, 1500, this);

        wheels.strafe(180, 800, TurnDirection.LEFT, tracker.get(0), this); //gears is 3
        //wheels.strafe(15, .5, TurnDirection.LEFT, gyro, this);
        //wheels.driveUntilWhite(-.09, opt, this);

        sleep(250);
        int red = color.red();
        int blue = color.blue();

        int distanceInBetween = 60;

        DbgLog.msg("[Phoenix - main] color blue0: " + blue);
        DbgLog.msg("[Phoenix - main] color red0: " + red);

        if (blue > 1) {
            wheels.strafe(2, 1700, TurnDirection.LEFT, gyro, this);
            DbgLog.msg("[Phoenix - main] color blue1: " + blue);
            DbgLog.msg("[Phoenix - main] color red1: " + red);
        } else {
            wheels.drive(5, Direction.FORWARD, 800, this);
            distanceInBetween += 5;

            sleep(250);
            red = color.red();
            blue = color.blue();

            DbgLog.msg("[Phoenix - main] color blue2: " + blue);
            DbgLog.msg("[Phoenix - main] color red2: " + red);
            if (blue > 1) {
                wheels.strafe(2, 1700, TurnDirection.LEFT, gyro, this);
            }
        }

        wheels.strafe(20, 1200, TurnDirection.RIGHT, gyro, this);
        wheels.drive(distanceInBetween, Direction.BACKWARD, 1800, this);

        boolean sawImage = wheels.strafe(170, 800, TurnDirection.LEFT, tracker.get(2), this);

        if(!sawImage) {
            wheels.strafe(17, 1000, TurnDirection.LEFT, gyro, this);
        }

        sleep(250);
        red = color.red();
        blue = color.blue();

        DbgLog.msg("[Phoenix - main] color blue3: " + blue);
        DbgLog.msg("[Phoenix - main] color red3: " + red);

        if (blue > 1) {
            wheels.strafe(2, 1700, TurnDirection.LEFT, gyro, this);
            DbgLog.msg("[Phoenix - main] color blue5: " + blue);
            DbgLog.msg("[Phoenix - main] color red5: " + red);
        } else {
            wheels.drive(5, Direction.FORWARD, 800, this);

            sleep(250);
            red = color.red();
            blue = color.blue();

            DbgLog.msg("[Phoenix - main] color blue6: " + blue);
            DbgLog.msg("[Phoenix - main] color red6: " + red);

            if (blue > 1) {
                wheels.strafe(2, 1700, TurnDirection.LEFT, gyro, this);
            }
        }

        wheels.strafe(10, 1200, TurnDirection.RIGHT, gyro, this);
        wheels.turnWithGyro(38, .7, TurnDirection.RIGHT, gyro, this);
        wheels.drive(60, Direction.FORWARD, 2100, this);
    }
}

