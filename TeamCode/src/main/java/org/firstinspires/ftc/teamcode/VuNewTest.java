package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by HwaA1 on 11/24/2016.
 */
@Autonomous(name = "VuTest", group = "none")
public class VuNewTest extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    VuforiaTrackables myPanda;


    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parms);

        myPanda = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        myPanda.get(0).setName("Wheels");
        myPanda.get(1).setName("Tools");
        myPanda.get(2).setName("Legos");
        myPanda.get(3).setName("Gears");

        myPanda.activate();

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        waitForStart();


        while(opModeIsActive()) {
            boolean one;
            if(((VuforiaTrackableDefaultListener) myPanda.get(2).getListener()).getPose() != null)
                one = true;
            else
                one = false;
            boolean two;
            if(((VuforiaTrackableDefaultListener) myPanda.get(1).getListener()).getPose() != null)
                two = true;
            else
                two = false;

            telemetry.addData("legos", String.valueOf(one));
            telemetry.addData("tools", String.valueOf(two));
            telemetry.update();
        }
    }
}
