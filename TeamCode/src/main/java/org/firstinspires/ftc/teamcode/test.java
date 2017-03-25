package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
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

public class test extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    VuforiaTrackables tracker;

    MecanumDriveTrain wheels;

    MyUtility myUtility;

    int pic = 0;

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

        wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", "leftwheels", "rightwheels", this);
        myUtility = new MyUtility();

        waitForStart();

        long startTime = System.currentTimeMillis();


        VuforiaTrackableDefaultListener image = (VuforiaTrackableDefaultListener) tracker.get(pic).getListener();
        OpenGLMatrix pos = image.getPose();
        VectorF position = pos.getTranslation();
        double y = position.get(2) * -1;

        DbgLog.msg("[Phoenix:ImageStrafe] Voltage=%6.3; ImageAngle=%d; Distance=7.4F", wheels.getVoltage(), myUtility.getImageAngle(tracker.get(pic)), y);

        double lastX = wheels.strafe(180, wheels.strafePowerToBeacon(), TurnDirection.LEFT, tracker.get(pic), this);

        for(int i = 0; i < 500; i++) {
            DbgLog.msg("[Phoenix:ImageStrafe] Voltage=%6.3; ImageAngle=%d; Distance=7.4F", wheels.getVoltage(), myUtility.getImageAngle(tracker.get(pic)), y);
        }


    }
}
