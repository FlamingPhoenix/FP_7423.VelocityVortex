package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Steve on 11/24/2016.
 */
@Autonomous(name="test Vuforia", group="test")
@Disabled()
public class TestVu extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaLocalizer vuforia;

    VuforiaTrackables trackObjects;

    private void initialize() {
        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parms);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        trackObjects = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        trackObjects.get(0).setName("Wheels");
        trackObjects.get(1).setName("Tools");
        trackObjects.get(2).setName("Legos");
        trackObjects.get(3).setName("Gears");
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        runtime.reset();
        trackObjects.activate();

        while (this.opModeIsActive()) {

            OpenGLMatrix pos = ((VuforiaTrackableDefaultListener) trackObjects.get(2).getListener()).getPose();

            if (pos != null) {
                VectorF p = pos.getTranslation();
                VectorF c3 = pos.getColumn(3);
                VectorF c2 = pos.getColumn(2);
                VectorF c1 = pos.getColumn(1);
                VectorF c0 = pos.getColumn(0);

                MatrixF m = pos.added(MatrixF.identityMatrix(4));

                Orientation orient = Orientation.getOrientation(m, AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
                float a1 = orient.firstAngle;
                float a2 = orient.secondAngle;
                float a3 = orient.thirdAngle;

                VectorF angleVector = pos.getColumn(0);
                VectorF directionVector = pos.getTranslation();

                float angleX = angleVector.get(0) * -1; // phone is upside down, so, x value is opposite
                float angleZ = angleVector.get(2) * -1; // we are using the back camera, so the z value is also reverse

                float directionX = directionVector.get(0) * -1;
                float directionZ = directionVector.get(2) * -1;

                float angle = (float) Math.toDegrees(Math.atan2(angleZ, angleX));
                float direction = (float) Math.toDegrees(Math.atan2(directionZ, directionX));

                float x = p.get(0) * -1f; //phone is sitting down (portrait), x is reverse.
                float y = p.get(2) * -1f; //phone is setting down, z is y and reverse (portrait) distance

                float c3x = c3.get(0);
                float c3y = c3.get(1);
                float c3z = c3.get(2);
                float c3w = c3.get(3);

                float c2x = c2.get(0);
                float c2y = c2.get(1);
                float c2z = c2.get(2);
                float c2w = c2.get(3);

                float c1x = c1.get(0);
                float c1y = c1.get(1);
                float c1z = c1.get(2);
                float c1w = c1.get(3);

                float c0x = c0.get(0);
                float c0y = c0.get(1);
                float c0z = c0.get(2);
                float c0w = c0.get(3);

                DbgLog.msg("[Vu] Saw %s", trackObjects.get(2).getName());
                DbgLog.msg("[Vu] x: %f", x);
                DbgLog.msg("[Vu] y: %f", y);

                DbgLog.msg("c2 x,y,z", "%5.3f %5.3f %5.3f", c2x, c2y, c2z);
                DbgLog.msg("c1 x,y,z", "%5.3f %5.3f %5.3f", c1x, c1y, c1z);
                DbgLog.msg("c0 x,y,z", "%5.3f %5.3f %5.3f", c0x, c0y, c0z);

                telemetry.addData("c2 x,y,z", "%5.3f %5.3f %5.3f", c2x, c2y, c2z);
                telemetry.addData("c1 x,y,z", "%5.3f %5.3f %5.3f", c1x, c1y, c1z);
                telemetry.addData("c0 x,y,z", "%5.3f %5.3f %5.3f", c0x, c0y, c0z);
                telemetry.addData("angle", "%5.3f", angle);
                telemetry.addData("direction", "%5.3f", direction);
                telemetry.addData("a1, a2, a3", "%2.3f %2.3f %2.3f", a1, a2, a3);
                telemetry.update();
            }
            this.idle();
        }
    }
}
