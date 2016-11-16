package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import FlamingPhoenix.*;

/**
 * Created by brand on 9/29/2016.
 */

@Autonomous(name = "Red_Auton", group = "Practice")
public class Practice_Auton extends LinearOpMode {

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
        while (gyro.isCalibrating())
            Thread.sleep(50);

        MecanumDriveTrain wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);

        gyro.calibrate();

        tracker.activate();

        waitForStart();

        wheels.drive(28,Direction.FORWARD, 1500, this);
        wheels.strafe(15, 1700, TurnDirection.LEFT, gyro, this);
        wheels.drive(34, Direction.FORWARD, 1500, this);

        wheels.strafe(200, 1200, TurnDirection.LEFT, (VuforiaTrackableDefaultListener) tracker.get(3).getListener(), this); //gears is 3
        //wheels.strafe(15, .5, TurnDirection.LEFT, gyro, this);
        //wheels.driveUntilWhite(-.09, opt, this);

        if (color.red() > 0) {
            wheels.strafe(2, 1700, TurnDirection.LEFT, gyro, this);
            DbgLog.msg("[Phoenix - main] color: " + color.blue());
        } else if (color.blue() > 0) {
            wheels.drive(6, Direction.FORWARD, 1400, this);
            if (color.red() > 0) {
                wheels.strafe(2, 1700, TurnDirection.LEFT, gyro, this);
                DbgLog.msg("[Phoenix] i see red");
            }
        }

        wheels.strafe(8, 1200, TurnDirection.RIGHT, gyro, this);
        wheels.drive(52, Direction.FORWARD, 1800, this);

        wheels.strafe(200, 600, TurnDirection.LEFT, (VuforiaTrackableDefaultListener) tracker.get(1).getListener(), this);

        if (color.red() > 0) {
            wheels.strafe(2, 1700, TurnDirection.LEFT, gyro, this);
            DbgLog.msg("[Phoenix - main] color: " + color.red());
        } else if (color.blue() > 0) {
            wheels.drive(6, Direction.FORWARD, 720, this);
            if (color.red() > 0) {
                wheels.strafe(2, 1700, TurnDirection.LEFT, gyro, this);
                DbgLog.msg("[Phoenix] i see red");
            }
        }
    }
}
