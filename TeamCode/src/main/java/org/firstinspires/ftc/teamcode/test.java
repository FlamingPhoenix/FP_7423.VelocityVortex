package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import FlamingPhoenix.MecanumDriveTrain;
import FlamingPhoenix.MyUtility;
import FlamingPhoenix.TurnDirection;

/**
 * Created by HwaA1 on 3/25/2017.
 */

@Autonomous(name = "TestStrafe", group = "none")

public class test extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    VuforiaTrackables tracker;

    MecanumDriveTrain wheels;

    MyUtility myUtility;

    int pic = 3;

    ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parms);
        tracker = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        tracker.activate();

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", "leftwheels", "rightwheels", gyro,this);
        myUtility = new MyUtility();

        gyro.calibrate();

        while(gyro.isCalibrating()){
            Thread.sleep(50);
        }

        waitForStart();

        VuforiaTrackableDefaultListener image =(VuforiaTrackableDefaultListener) tracker.get(pic).getListener();;

        for(int x = 0; x < 100; x++) {
            image = (VuforiaTrackableDefaultListener) tracker.get(pic).getListener();
            if(image != null)
                break;
        }

        OpenGLMatrix pos = image.getPose();
        VectorF position = pos.getTranslation();
        double y = position.get(2) * -1;
        double x = position.get(0) * -1;

        DbgLog.msg("[Phoenix:ImageStrafe] Voltage=%6.3f; ImageAngle=%d; Distance=%7.4f; imageX=%7.4f", wheels.getVoltage(), myUtility.getImageAngle(tracker.get(pic)), y, x);

        long startTime = System.currentTimeMillis();

        int heading = gyro.getIntegratedZValue();

        double lastX = wheels.strafe(100, wheels.strafePowerToBeacon(), TurnDirection.LEFT, tracker.get(pic), heading, this);

        for(int i = 0; i < 500; i++) {
            pos = image.getPose();
            if(pos != null) {
                position = pos.getTranslation();
                y = position.get(2) * -1;
                x = position.get(0) * -1;
                long timeElasped = Math.abs(System.currentTimeMillis() - startTime);
                DbgLog.msg("[Phoenix:ImageStrafe] Voltage=%6.3f; Time=%d; ImageAngle=%d; Distance=%7.4f; imageX=%7.4f", wheels.getVoltage(), timeElasped, myUtility.getImageAngle(tracker.get(pic)), y, x);
            }
            else {
                DbgLog.msg("[Phoenix:ImageStrafe] Can't see the image anymore");
            }
        }


    }
}
