package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import FlamingPhoenix.Direction;
import FlamingPhoenix.MecanumDriveTrain;
import FlamingPhoenix.MyUtility;
import FlamingPhoenix.TurnDirection;

/**
 * Created by HwaA1 on 11/12/2016.
 */

@Autonomous(name = "Blue_Auton", group = "Autonomous")

public class Blue_Auton extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    VuforiaTrackables tracker;

    DcMotor shooter;
    DcMotor collecter;

    Servo stopper;
    Servo pusher;
    Servo poker;

    ColorSensor color;

    MecanumDriveTrain wheels;

    OpticalDistanceSensor opt;
    ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", "leftwheels", "rightwheels",this);

        shooter = hardwareMap.dcMotor.get("farriswheel");
        stopper = hardwareMap.servo.get("stopper");
        poker = hardwareMap.servo.get("poker");

        collecter = hardwareMap.dcMotor.get("collector");

        pusher = hardwareMap.servo.get("pusher");

        pusher.setPosition(.5);
        stopper.setPosition(.75);

        color = hardwareMap.colorSensor.get("color");
        color.enableLed(false);

        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parms);

        tracker = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        tracker.activate();

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        gyro.resetZAxisIntegrator();

        opt = hardwareMap.opticalDistanceSensor.get("opt");

        while (gyro.isCalibrating() && this.opModeIsActive())
            Thread.sleep(50);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.idle();
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMaxSpeed(2500);

        gyro.resetZAxisIntegrator();
        gyro.calibrate();

        while(gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        waitForStart();

        //collecter.setPower(.5);
        //sleep(80);
        //collecter.setPower(0);
        Thread.sleep(5);
        shooter.setPower(1);

        wheels.strafe(6, 0.6, TurnDirection.LEFT, this);

        Thread.sleep(1500);

        stopper.setPosition(.20); //Shoot
        Thread.sleep(250);
        stopper.setPosition(.75); //Stop shooting
        Thread.sleep(1500);
        stopper.setPosition(0.20); //Shoot
        Thread.sleep(500);
        stopper.setPosition(.75); //Stop shooting

        wheels.strafe(14, 0.8, TurnDirection.LEFT, this);

        shooter.setPower(0);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.idle();
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.idle();

        wheels.drive(20, Direction.FORWARD, 0.4, 5, this);
        this.sleep(200);

        int angleBefore= gyro.getIntegratedZValue();
        int degreesNeeded = Math.abs(-88 - angleBefore);
        wheels.turnWithGyro(degreesNeeded, wheels.turnPower(), TurnDirection.RIGHT, gyro, this);
        int angleAfter = gyro.getIntegratedZValue();

        DbgLog.msg("[Phoenix] angleBefore= %d, degreeNeeded= %d, angleAfter= %d", angleBefore, degreesNeeded, angleAfter);

        wheels.drive(27, Direction.BACKWARD, 0.3, 5, this);

        wheels.driveUntilImage(15, .15, Direction.BACKWARD, tracker.get(0), this);

        int angle = MyUtility.getImageAngle(tracker.get(0));
        DbgLog.msg("[Phoenix] angle2: " + angle);

        int adjAngle = 0;
        int heading = gyro.getIntegratedZValue();
        int nowsHeading = gyro.getIntegratedZValue(); //the Gyro heading before adjusting robot based on image angle

        if (angle != -999) {//saw image and found the angle of the image
            int turnAngle = Math.abs(90 - angle);
            adjAngle = 90 - angle;

            heading = gyro.getIntegratedZValue() - adjAngle; //this is the target Gyro angle that we need to turn to for later steps

            if(angle <= 85) {
                DbgLog.msg("[Phoenix:Step 3 - AdjustHeading] Adjust to turn RIGHT by %d degree based on image angle of %d", turnAngle, angle);

                if (turnAngle <= 10)
                    makeMinorTurn(heading);
                else
                    wheels.turnWithGyro(turnAngle, .2, TurnDirection.RIGHT, gyro, this);
            }
            else if(angle >= 95) {
                DbgLog.msg("[Phoenix:Step 3 AdjustHeading] Adjust to turn left by %d degree based on image angle of %d", turnAngle, angle);

                if (turnAngle <= 10)
                    makeMinorTurn(heading);
                else
                    wheels.turnWithGyro(turnAngle, .2, TurnDirection.LEFT, gyro, this);
            }
        }

        double lastX = wheels.strafe(180, wheels.strafePowerToBeacon(), TurnDirection.LEFT, tracker.get(0), this);
        float imageX;

        int endHeading = gyro.getIntegratedZValue();
        int turningAngle = heading - endHeading;
        TurnDirection d;

        float adjustmentDistance;
        Direction adjustDirection;

        int didWeGoBack = 0;

        if(lastX != -9999) {
            if (turningAngle < 0)
                d = TurnDirection.RIGHT;
            else
                d = TurnDirection.LEFT;

            if (d == TurnDirection.LEFT)
                DbgLog.msg("[Phoenix:Step 4 Beacon Adjustment 1] At beacon, Reached beacon turn LEFT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
            else
                DbgLog.msg("[Phoenix:Step 4 Beacon Adjustment 1] At beacon, Reached beacon turn RIGHT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);

            if (Math.abs(turningAngle) > 5) { //The robot is not parallel to the beacon, need to adjust

                if(turningAngle > 10)
                    wheels.turnWithGyro(Math.abs(turningAngle), .2, d, gyro, this);
                else
                    makeMinorTurn(heading);

                DbgLog.msg("[Phoenix:Step 4Beacon Adjustment 1] At beacon, performed Reached beacon turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
            }

            imageX = MyUtility.getImageXPosition(tracker.get(0));
            DbgLog.msg("[Phoenix] imageX=%9.3f", imageX);
            if ((imageX == -9999) && (lastX != -9999)) { //can't see image and we got the X during strafing
                imageX = (float) lastX;
                DbgLog.msg("[Phoenix] Can't see image, use last known imageX=%9.3f", imageX);
            } else if (imageX == -9999) {
                imageX = 0;
            }

            if (imageX < 0) {// We need to move back (to the left of image a bit)
                adjustDirection = Direction.FORWARD;
            } else if (imageX > 0) {
                adjustDirection = Direction.BACKWARD;
            } else {
                adjustDirection = Direction.FORWARD;
            }

            adjustmentDistance = Math.abs(((imageX * 100.0f)) / 254.0f) / 10.0f;
            if (adjustDirection == Direction.FORWARD)
                adjustmentDistance = adjustmentDistance + .5f; //need to move forward a bit more to handle the strafing problem

            if (adjustDirection == Direction.BACKWARD)
                DbgLog.msg("[Phoenix] Beacon X position adjustment backward %7.3f and image X %7.3f", adjustmentDistance, imageX);
            else
                DbgLog.msg("[Phoenix] Beacon X position adjustment forward %7.3f and imageX %7.3f", adjustmentDistance, imageX);

            if (adjustmentDistance >= 1) {
                DbgLog.msg("[Phoenix] Performed Beacon X position adjustment %7.3f", adjustmentDistance);
                wheels.drive((int) adjustmentDistance, adjustDirection, 0.3, 2, this);
            }

            boolean pushAnyway = false;  //if see blue on one side, then, the other side got to be red
            if (color.blue() <= 1) {
                DbgLog.msg("[Phoenix] Can't see red, move back 5 inches");

                if (color.red() > 1) //if see blue on one side, then, the other side got to be red
                    pushAnyway = true;

                wheels.drive(7, Direction.BACKWARD, 0.2, 5, this);
                didWeGoBack = 5;
            }

            if ((color.blue() > 1) || pushAnyway)  { //sees the blue side or the other side is red
                pusher.setPosition(0);
                Thread.sleep(1500);
                wheels.strafe(1, .5, 1, TurnDirection.LEFT, this);
                Thread.sleep(200);

                pusher.setPosition(1);
                Thread.sleep(500);

                wheels.strafe(14, .65, TurnDirection.RIGHT, this);
                pusher.setPosition(.5);
            } else {
                wheels.strafe(13, .65, TurnDirection.RIGHT, this);
                pusher.setPosition(.5);
            }
        }

        //Now go for 2nc Beacon, Adjust the angle first
        Thread.sleep(50);
        endHeading = gyro.getIntegratedZValue();
        turningAngle = heading - endHeading;

        if(turningAngle < 0)
            d = TurnDirection.RIGHT;
        else
            d = TurnDirection.LEFT;

        if (d == TurnDirection.LEFT)
            DbgLog.msg("[Phoenix:Step 5 Beacon Adjustment 2] Leaving first beacon, Reached beacon turn LEFT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
        else
            DbgLog.msg("[Phoenix:Step 5 Beacon Adjustment 2] Leaving first beacon, Reached beacon turn RIGHT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);

        if (Math.abs(turningAngle) > 3){

            if (Math.abs(turningAngle) < 10)
                makeMinorTurn(heading);
            else
                wheels.turnWithGyro(Math.abs(turningAngle), .2, d, gyro, this);

            DbgLog.msg("[Phoenix:Step 5 Beacon Adjustment 2] Leaving first beacon, performed Reached beacon turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
        }

        wheels.drive(45 - didWeGoBack, Direction.BACKWARD, wheels.drivePowerToBeacon2(), 4, this);

        wheels.driveUntilImage(15, 0.12, Direction.BACKWARD, tracker.get(2), this);

        angle = MyUtility.getImageAngle(tracker.get(2));
        DbgLog.msg("[Phoenix] 2nd beeacon angle: " + angle);

        endHeading = gyro.getIntegratedZValue(); //Read the current Gyro heading
        if (angle != -999) {//saw image and found the angle of the image
            turningAngle = heading - endHeading;

            if (turningAngle < 0)
                d = TurnDirection.RIGHT;
            else
                d = TurnDirection.LEFT;

            if (Math.abs(turningAngle) >= 4) {
                if (Math.abs(turningAngle) > 10)
                    wheels.turnWithGyro(Math.abs(turningAngle), .2, d, gyro, this);
                else
                    makeMinorTurn(Math.abs(turningAngle), d);

                if (d == TurnDirection.LEFT)
                    DbgLog.msg("[Phoenix:Step 6] Adjust to turn left by %d degree based on 2nd image angle of %d", Math.abs(turningAngle), angle);
                else
                    DbgLog.msg("[Phoenix:Step 6] Adjust to turn right by %d degree based on 2nd image angle of %d", Math.abs(turningAngle), angle);

            }
        }

        lastX = wheels.strafe(180, wheels.strafePowerToBeacon(), TurnDirection.LEFT, tracker.get(2), this);

        endHeading = gyro.getIntegratedZValue();
        turningAngle = heading - endHeading;

        if(turningAngle < 0)
            d = TurnDirection.RIGHT;
        else
            d = TurnDirection.LEFT;

        if (d == TurnDirection.LEFT)
            DbgLog.msg("[Phoenix:Step 7] At 2nd beacon, Reached beacon turn LEFT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
        else
            DbgLog.msg("[Phoenix:Step 7] At 2nd beacon, Reached beacon turn RIGHT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);

        if (Math.abs(turningAngle) > 5){ //robot is not parallel by more than 5 degree, adjust
            if(turningAngle > 10)
                wheels.turnWithGyro(Math.abs(turningAngle), .2, d, gyro, this);
            else
                makeMinorTurn(heading);

            DbgLog.msg("[Phoenix:Step 6] At beacon, performed Reached beacon turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
        }

        imageX = MyUtility.getImageXPosition(tracker.get(2));
        DbgLog.msg("[Phoenix] 2nd imageX=%9.3f", imageX);
        if ((imageX == -9999) && (lastX != -9999)) { //can't see image and we got the X during strafing
            imageX = (float) lastX;
            DbgLog.msg("[Phoenix] Can't see 2nd image, use last known imageX=%9.3f", imageX);
        }
        else if (imageX == -9999) {
            imageX = 0;
        }

        if (imageX < 0) {// We need to move back (to the left of image a bit)
            adjustDirection = Direction.FORWARD;
        }
        else if (imageX > 0) {
            adjustDirection = Direction.BACKWARD;
        }
        else {
            adjustDirection = Direction.FORWARD;
        }

        adjustmentDistance = Math.abs(( (imageX * 100.0f))/ 254.0f) / 10.0f ;
        if (adjustDirection == Direction.FORWARD)
            adjustmentDistance = adjustmentDistance + .5f; //need to move forward a bit more to handle the strafing problem

        if (adjustDirection == Direction.BACKWARD)
            DbgLog.msg("[Phoenix] 2nd Beacon X position adjustment backward %7.3f and image X %7.3f", adjustmentDistance, imageX);
        else
            DbgLog.msg("[Phoenix] 2nd Beacon X position adjustment forward %7.3f and imageX %7.3f", adjustmentDistance, imageX);

        if(adjustmentDistance >= 1) {
            DbgLog.msg("[Phoenix] Performed 2nd Beacon X position adjustment %7.3f", adjustmentDistance);
            wheels.drive((int) adjustmentDistance, adjustDirection, 0.2, 2, this);
        }

        boolean pushAnyway = false;  //if see blue on one side, then, the other side got to be red
        boolean turnSize = false;
        if(color.blue() <= 1) {
            DbgLog.msg("[Phoenix:Beacon Distance Adjustment 2] Can't see red, move back 6 inches");

            if (color.red() > 1)
                pushAnyway = true; //if see blue on one side, then, the other side got to be red

            wheels.drive(7, Direction.BACKWARD, 0.2, 5, this);
            turnSize = true;
        }

        if ((color.blue() > 1) || pushAnyway) { //sees the blue side or otherside is red
            pusher.setPosition(0);
            Thread.sleep(1000);
            wheels.strafe(2, .5, 1, TurnDirection.LEFT, this);

            pusher.setPosition(1);
            wheels.strafe(7, .5, 5, TurnDirection.RIGHT, this);
            pusher.setPosition(0.5);
        }
        else {
            wheels.strafe(8, .5, TurnDirection.RIGHT, this);
        }

        angleBefore = gyro.getIntegratedZValue();
        if(turnSize)
            degreesNeeded = Math.abs(-55 - angleBefore);
        else
            degreesNeeded = Math.abs(-60 - angleBefore);

        wheels.turnWithGyro(degreesNeeded, .5, TurnDirection.RIGHT, gyro, this);
        wheels.drive(62, Direction.FORWARD, .8, 6, this);
    }

    //Make small angle turn
    void makeMinorTurn(int targetGyroHeading) {
        int currentHeading = gyro.getIntegratedZValue();
        int turningAngle = targetGyroHeading - currentHeading;

        TurnDirection d;
        if(turningAngle < 0)
            d = TurnDirection.RIGHT;
        else
            d = TurnDirection.LEFT;

        makeMinorTurn(turningAngle, d);
    }
    //Make small angle turn
    void makeMinorTurn(int turningAngle, TurnDirection direction) {

        turningAngle = Math.abs(turningAngle);

        if (turningAngle < 5)
            wheels.turnAjdustment(0.2, direction, this);
        else if (turningAngle < 8)
            wheels.turnAjdustment(0.22, direction, this);
        else if (turningAngle < 10)
            wheels.turnAjdustment(0.26, direction, this);
        else
            wheels.turnAjdustment(0.35, direction, this);
    }
}

