package org.firstinspires.ftc.teamcode;

import android.database.sqlite.SQLiteBindOrColumnIndexOutOfRangeException;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
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

    private Servo Phone;
    VuforiaTrackables myPanda;

    private void initialize() {
        backLeft = hardwareMap.dcMotor.get("backleft");
        backRight = hardwareMap.dcMotor.get("backright");
        frontLeft = hardwareMap.dcMotor.get("frontleft");
        frontRight = hardwareMap.dcMotor.get("frontright");

        //Phone = hardwareMap.servo.get("phone");
        //Phone.setPosition(0.8);

        //Left Side motors should rotate opposite of right side motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parms);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        myPanda = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        myPanda.get(0).setName("Wheels");
        myPanda.get(1).setName("Tools");
        myPanda.get(2).setName("Legos");
        myPanda.get(3).setName("Gears");
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

//        myPanda.get(1).setName("Legos");

        waitForStart();
        runtime.reset();

        myPanda.activate();

        while (opModeIsActive()) {

            OpenGLMatrix pos = ((VuforiaTrackableDefaultListener) myPanda.get(1).getListener()).getPose();

            if (pos != null)  //The front of folder is visible
                telemetry.addData("Tools Visible", "true");
            else {
                pos = ((VuforiaTrackableDefaultListener) myPanda.get(2).getListener()).getPose(); //The front is not visible, look for the back
                if (pos != null)
                    telemetry.addData("Legos Visible", "true");
                else
                    telemetry.addData("Folder Tools & Legos Visible", "false");
            }

            if (pos != null) {
                VectorF translation = pos.getTranslation();

                float x = translation.get(0) * -1f; //phone is sitting down (portrait), x is reverse.
                float y = translation.get(2) * -1f; //phone is setting down, z is y and reverse (portrait) distance
                float z = translation.get(1); //vertical

                float newY = (float) Math.sqrt((Math.pow(x, 2) + Math.pow(y, 2))); //distance of the object on the same plain as the phone
                float d = (float) Math.toDegrees(Math.atan2((double)x, (double)y));  //get the angle

                float flp = 0; //front left power
                float frp = 0; //front right power
                float blp = 0;
                float brp = 0;

                float td = 600; //Target Distance, in millimeter

                //The following is change the left and right power to turn right or left to follow the object
                if (Math.abs(d) > 10) { //only move if the target is outside of plus/minus 10 degree angle
                    flp = 0.15f * (d/Math.abs(d));
                    blp = flp;

                    frp = flp * -1;
                    brp = flp * -1;
                }
                else {
                    //The following is to adjust the power to keep a distance from the robot
                    DbgLog.msg("[Phoenix] Control Distance");

                    float diff = newY - td;
                    DbgLog.msg("[Phoenix] Diff="+ Float.toString(diff));
                    if (Math.abs(diff) > 100) {//distance has changed by more than 100 millimeter (10 centermeter)

                        float ap = diff * 0.0020f;

                        if (ap > 0.45)
                            ap = 0.45f;
                        else if (ap < -0.45)
                            ap = -0.45f;

                        flp = flp - ap; //move forward or background based on the change in distance
                        blp = blp + ap;
                        frp = frp + ap;
                        brp = brp - ap;

                        DbgLog.msg("[Phoenix] flp-before-rationalize="+ Float.toString(flp));
                        DbgLog.msg("[Phoenix] frp-before-rationalize="+ Float.toString(frp));
                        DbgLog.msg("[Phoenix] blp-before-rationalize="+ Float.toString(blp));
                        DbgLog.msg("[Phoenix] brp-before-rationalize="+ Float.toString(brp));

                        if ((Math.abs(frp) > 1) || (Math.abs(flp) > 1) || (Math.abs(brp) > 1) || (Math.abs(blp) > 1)) { //either left power or right power are greater than 1, too much power
                            //The following is to adjust the power proportionally between left and right
                            float lrp = (float) myMax(Math.abs(flp), Math.abs(frp), Math.abs(brp), Math.abs(blp));

                            flp = flp / lrp; //adjust the space so it will not be more han 1
                            frp = frp / lrp;
                            blp = blp / lrp;
                            brp = brp / lrp;

                            DbgLog.msg("[Phoenix] flp-after-rationalize="+ Float.toString(flp));
                            DbgLog.msg("[Phoenix] frp-after-rationalize="+ Float.toString(frp));
                            DbgLog.msg("[Phoenix] blp-after-rationalize="+ Float.toString(blp));
                            DbgLog.msg("[Phoenix] brp-after-rationalize="+ Float.toString(brp));
                        }
                    }
                }

                flp = Range.clip(flp, -1, 1);
                frp = Range.clip(frp, -1, 1);
                blp = Range.clip(blp, -1, 1);
                brp = Range.clip(brp, -1, 1);


                DbgLog.msg("[Phoenix] flp-after-range="+ Float.toString(flp));
                DbgLog.msg("[Phoenix] frp-after-range="+ Float.toString(frp));
                DbgLog.msg("[Phoenix] blp-after-range="+ Float.toString(blp));
                DbgLog.msg("[Phoenix] brp-after-range="+ Float.toString(brp));

                frontLeft.setPower(flp);
                backLeft.setPower(blp);
                frontRight.setPower(frp);
                backRight.setPower(brp);

                telemetry.addData("Translation - newY", newY);
                telemetry.addData("Translation - x", x);
                telemetry.addData("Translation - y", y);
                telemetry.addData("Translation - z", z);
                telemetry.addData("Degree", d);
            }
            else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }

            telemetry.update();
            idle();
        }
    }

    private float myMax(float v1, float v2, float v3, float v4) {
        if ((v1 >= v2) && (v1 >= v3) && (v1 >= v4))
            return v1;

        if ((v2 >= v1) && (v2 >= v3) && (v2 >= v4))
            return v2;


        if ((v3 >= v1) && (v3 >= v2) && (v3 >= v4))
            return v3;

        return v4;
    }

    /*
    //get the phone's angle
    private int phoneAngle() {

        //0.8 is 0, 1 is the max
        double s = Phone.getPosition();

        return (int) Math.round(((s - 0.8) / 1) * 175);
    }

    private void moveUp() {
        double n = Phone.getPosition() + 0.005;
        if (n <= 1)
            Phone.setPosition(n);
    }
    private void moveDown() {
        double n = Phone.getPosition() - 0.005;
        if (n >= 0.75)
            Phone.setPosition(n);
    }

    */
}
