package org.firstinspires.ftc.teamcode;

import android.database.sqlite.SQLiteBindOrColumnIndexOutOfRangeException;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Steve on 10/10/2016.
 */

@Autonomous(name="Try Tracking", group="Trial")
public class VuAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaLocalizer vuforia;

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    @Override
    public void runOpMode() throws InterruptedException {

        backLeft = hardwareMap.dcMotor.get("backleft");
        backRight = hardwareMap.dcMotor.get("backright");
        frontLeft = hardwareMap.dcMotor.get("frontleft");
        frontRight = hardwareMap.dcMotor.get("frontright");

        //Left Side motors should rotate opposite of right side motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parms);

        VuforiaTrackables myPanda = vuforia.loadTrackablesFromAsset("Trial_OT");

        myPanda.get(0).setName("spaceshuttle");

        waitForStart();
        runtime.reset();

        myPanda.activate();

        while (opModeIsActive()) {

            OpenGLMatrix pos = ((VuforiaTrackableDefaultListener) myPanda.get(0).getListener()).getPose();

            if (pos != null) {
                VectorF translation = pos.getTranslation();

                float x = translation.get(0); //phone is sitting up (portrait)
                float y = translation.get(2) * -1f; //phone is setting up, z is y and reverse (portrait)

                float d = (float) Math.toDegrees(Math.atan2((double)x, (double)y));

                float p;
                if (Math.abs(d) < 10) //only move if the target is outside of plus/minus 10 degree angle.
                    p = Range.clip(d * 0.02f, -1f, 1f);
                else
                    p = 0;

                frontLeft.setPower(p);
                backLeft.setPower(p);
                frontRight.setPower(p * -1);
                backRight.setPower(p * -1);


                telemetry.addData("Translation - x", x);
                telemetry.addData("Translation - y", y);
                telemetry.addData("Translation - z", translation.get(2));
                telemetry.addData("Degree", d);
            }
            else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);

                telemetry.addData("Visible:", "false");
            }

            telemetry.update();
            idle();
        }
    }
}
