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

        Phone = hardwareMap.servo.get("phone");
        Phone.setPosition(0.9);

        //Left Side motors should rotate opposite of right side motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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

                float x = translation.get(0); //phone is sitting up (portrait), distance from the object.
                float y = translation.get(2) * -1f; //phone is setting up, z is y and reverse (portrait) distance
                float z = translation.get(1) * -1f; //vertical

                float newY = (float) Math.sqrt((Math.pow(y, 2) - Math.pow(z, 2))); //distance of the object the same plain as the phone
                float d = (float) Math.toDegrees(Math.atan2((double)x, (double)newY));  //get the angle

                float lp; //left power
                float rp; //right power

                //The following is change the left and right power to turn right or left to follow the object
                if (Math.abs(d) > 5) { //only move if the target is outside of plus/minus 10 degree angle
                    lp = Range.clip(d * 0.015f, -1f, 1f);
                    rp = lp * -1;
                }
                else {
                    lp = 0;
                    rp = 0;

                    //The following is to adjust the power to keep a distance from the robot

                    DbgLog.msg("[Phoenix] Control Distance");

                    float td = 600; //Target Distance, in millimeter

                    float diff = newY - td;
                    DbgLog.msg("[Phoenix] Diff="+ Float.toString(diff));
                    if (Math.abs(diff) > 100) {//distance has changed by more than 100 millimeter (10 centermeter)

                        lp = lp + diff * 0.002f; //move forward or background based on the change in distance
                        rp = rp + diff * 0.002f;

                        DbgLog.msg("[Phoenix] lp-before-rationalize="+ Float.toString(lp));
                        DbgLog.msg("[Phoenix] rp-before-rationalize="+ Float.toString(rp));

                        if ((Math.abs(rp) > 1) || (Math.abs(lp) > 1)) { //either left power or right power are greater than 1, too much power
                            //The following is to adjust the power proportionally between left and right
                            float lrp = (float) Math.max(Math.abs(lp), Math.abs(rp));

                            lp = lp / lrp; //adjust the space so it will not be more han 1
                            rp = rp / lrp;

                            DbgLog.msg("[Phoenix] lp-after-rationalize="+ Float.toString(lp));
                            DbgLog.msg("[Phoenix] rp-after-rationalize="+ Float.toString(rp));
                        }
                    }

                }

                lp = Range.clip(lp, -1, 1);
                rp = Range.clip(rp, -1, 1);

                DbgLog.msg("[Phoenix] lp-after-range="+ Float.toString(lp));
                DbgLog.msg("[Phoenix] rp-after-range="+ Float.toString(rp));

                frontLeft.setPower(lp);
                backLeft.setPower(lp);
                frontRight.setPower(rp);
                backRight.setPower(rp);

                telemetry.addData("lp", lp);
                telemetry.addData("rp", rp);
                telemetry.addData("Translation - newY", newY);
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
}
