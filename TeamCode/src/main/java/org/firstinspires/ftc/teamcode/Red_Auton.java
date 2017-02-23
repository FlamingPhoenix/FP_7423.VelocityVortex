package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.FlamingPhoenix.*;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import FlamingPhoenix.*;
import FlamingPhoenix.MecanumDriveTrain;
import FlamingPhoenix.TurnDirection;

/**
 * Created by brand on 9/29/2016.
 */

@Autonomous(name = "Red_Auton", group = "Practice")
public class Red_Auton extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    VuforiaTrackables tracker;

    DcMotor shooter;

    Servo stopper;

    ColorSensor color;

    OpticalDistanceSensor opt;

    Servo pusher;

    DcMotor collecter;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.dcMotor.get("farriswheel");
        stopper = hardwareMap.servo.get("stopper");
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

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.resetZAxisIntegrator();
        gyro.resetZAxisIntegrator();
        gyro.calibrate();

        opt = hardwareMap.opticalDistanceSensor.get("opt");

        MecanumDriveTrain wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.idle();
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMaxSpeed(2450);

        while(gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        waitForStart();

        collecter.setPower(.5);
        sleep(80);
        collecter.setPower(0);
        shooter.setPower(1);

        wheels.strafe(6, 0.7, TurnDirection.LEFT, this);

        Thread.sleep(600);

        stopper.setPosition(.20); //Shoot
        Thread.sleep(250);
        stopper.setPosition(.75); //Stop shooting
        Thread.sleep(900);
        stopper.setPosition(0.20); //Shoot
        Thread.sleep(500);
        stopper.setPosition(.75); //Stop shooting

        wheels.strafe(14, 0.7, TurnDirection.LEFT, this);

        shooter.setPower(0);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.idle();
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.idle();

        wheels.drive(35, Direction.BACKWARD, 0.4, 6, this);
        this.sleep(200);

        int angleBefore = gyro.getIntegratedZValue();
        int degreesNeeded = Math.abs(88 - angleBefore);
        wheels.turnWithGyro(degreesNeeded, .25, TurnDirection.LEFT, gyro, this);
        int angleAfter = gyro.getIntegratedZValue();

        DbgLog.msg("[Phoenix] angleBefore= %d, degreeNeeded= %d, angleAfter= %d", angleBefore, degreesNeeded, angleAfter);

        wheels.drive(15, Direction.FORWARD, 0.3, 5, this);

        wheels.driveUntilImage(15, .15, Direction.FORWARD, tracker.get(3), this);

        //Get the first image angle
        int angle = MyUtility.getImageAngle(tracker.get(3));

        int adjAngle = 0;
        int heading = gyro.getIntegratedZValue();
        int nowsHeading = gyro.getIntegratedZValue();

        if (angle != -999) {//saw image and found the angle of the image
            int turnAngle = Math.abs(90 - angle);
            adjAngle = 90 - angle;

            heading = gyro.getIntegratedZValue() - adjAngle;

            if(angle < 85) {
                DbgLog.msg("[Phoenix:AdjustHeading] Adjust to turn RIGHT by %d degree based on image angle of %d", turnAngle, angle);
                wheels.turnWithGyro(turnAngle, .25, TurnDirection.RIGHT, gyro, this);
            }
            else if(angle > 95) {
                DbgLog.msg("[Phoenix:AdjustHeading] Adjust to turn left by %d degree based on image angle of %d", turnAngle, angle);
                wheels.turnWithGyro(turnAngle, .25, TurnDirection.LEFT, gyro, this);
            }
        }

        DbgLog.msg("[Phoenix:CalculateHeading] newNeededGyroHeading: %d. adjAngle: %d, imageAngle: %d, previousHeading %d", heading, adjAngle, angle, nowsHeading);

        wheels.resetMotorSpeed();
        double lastX = wheels.strafe(180, 0.8, TurnDirection.LEFT, tracker.get(3), this);
        DbgLog.msg("[Phoenix:ApproachImage 1] lastX After Strafe = " + lastX);
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
                DbgLog.msg("[Phoenix:Beacon Adjustment 1] At beacon, Reached beacon turn LEFT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
            else
                DbgLog.msg("[Phoenix:Beacon Adjustment 1] At beacon, Reached beacon turn RIGHT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);

            if (Math.abs(turningAngle) > 5) { //The robot is not parallel to the beacon, need to adjust
                wheels.turnWithGyro(Math.abs(turningAngle), .25, d, gyro, this);
                DbgLog.msg("[Phoenix:Beacon Adjustment 1] At beacon, performed Reached beacon turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
            }

            imageX = MyUtility.getImageXPosition(tracker.get(3));
            DbgLog.msg("[Phoenix:ApproachImage 1] imageX=%9.3f", imageX);
            if ((imageX == -9999) && (lastX != -9999)) { //can't see image and we got the X during strafing
                imageX = (float) lastX;
                DbgLog.msg("[Phoenix:ApproachImage 1] Can't see image, use last known imageX=%9.3f", imageX);
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
                adjustmentDistance = adjustmentDistance + 1.0f; //need to move forward a bit more to handle the strafing problem

            if (adjustDirection == Direction.BACKWARD)
                DbgLog.msg("[Phoenix: Beacon Distance Adjustment] Beacon X position adjustment backward %7.3f and image X %7.3f", adjustmentDistance, imageX);
            else
                DbgLog.msg("[Phoenix: Beacon Distance Adjustment] Beacon X position adjustment forward %7.3f and imageX %7.3f", adjustmentDistance, imageX);

            if (adjustmentDistance >= 1) {
                DbgLog.msg("[Phoenix: Beacon Distance Adjustment] Performed Beacon X position adjustment %7.3f", adjustmentDistance);
                wheels.drive((int) adjustmentDistance, adjustDirection, 0.25, 3, this);
            }

            boolean pushAnyway = false;  //if see blue on one side, then, the other side got to be red
            if (color.red() <= 1) {
                DbgLog.msg("[Phoenix] Can't see red, move back 5 inches");

                if (color.blue() > 1) //if see blue on one side, then, the other side got to be red
                    pushAnyway = true;

                wheels.drive(7, Direction.BACKWARD, 0.25, 5, this);
                didWeGoBack = 5;
            }

            if ((color.red() > 1) || pushAnyway)  { //sees the red side or the other side is blue
                pusher.setPosition(0);
                Thread.sleep(1000);
                wheels.strafe(1, .8, TurnDirection.LEFT, this);
                Thread.sleep(200);

                pusher.setPosition(1);
                Thread.sleep(500);

                wheels.strafe(12, .8, TurnDirection.RIGHT, this);
                pusher.setPosition(.5);
            } else {
                wheels.strafe(12, .8, TurnDirection.RIGHT, this);
                pusher.setPosition(.5);
            }
        }
        //First beacon (Gears) is complete

        //Not yet, before go for 2nc Beacon, Adjust the angle first
        endHeading = gyro.getIntegratedZValue();
        turningAngle = heading - endHeading;

        if(turningAngle < 0)
            d = TurnDirection.RIGHT;
        else
            d = TurnDirection.LEFT;

        if (d == TurnDirection.LEFT)
            DbgLog.msg("[Phoenix: Beacon Adjustment 2] Leaving first beacon, Reached beacon turn LEFT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
        else
            DbgLog.msg("[Phoenix: Beacon Adjustment 2] Leaving first beacon, Reached beacon turn RIGHT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);

        if (Math.abs(turningAngle) > 5){
            wheels.turnWithGyro(Math.abs(turningAngle), .25, d, gyro, this);
            DbgLog.msg("[Phoenix: Beacon Adjustment 2] Leaving first beacon, performed Reached beacon turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
        }

        //Now, got to 2nd beacon/image
        wheels.drive(49 + didWeGoBack, Direction.FORWARD, 0.4, 4, this);

        wheels.driveUntilImage(15, 0.15, Direction.FORWARD, tracker.get(1), this);

        angle = MyUtility.getImageAngle(tracker.get(1));
        DbgLog.msg("[Phoenix] 2nd beeacon angle: " + angle);

        if (angle != -999) {//saw image and found the angle of the image
            int turnAngle = Math.abs(90 - angle);

            if(angle < 85) {
                DbgLog.msg("[Phoenix] Adjust to turn RIGHT by %d degree based on 2nd image angle of %d", turnAngle, angle);
                wheels.turnWithGyro(turnAngle, .25, TurnDirection.RIGHT, gyro, this);
            }
            else if(angle > 95) {
                DbgLog.msg("[Phoenix] Adjust to turn left by %d degree based on 2nd image angle of %d", turnAngle, angle);
                wheels.turnWithGyro(turnAngle, .25, TurnDirection.LEFT, gyro, this);
            }
        }

        heading = gyro.getIntegratedZValue();
        wheels.resetMotorSpeed();
        lastX = wheels.strafe(180, 0.8, TurnDirection.LEFT, tracker.get(1), this);

        endHeading = gyro.getIntegratedZValue();
        turningAngle = heading - endHeading;

        if(turningAngle < 0)
            d = TurnDirection.RIGHT;
        else
            d = TurnDirection.LEFT;

        if (d == TurnDirection.LEFT)
            DbgLog.msg("[Phoenix] At 2nd beacon, Reached beacon turn LEFT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
        else
            DbgLog.msg("[Phoenix] At 2nd beacon, Reached beacon turn RIGHT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);

        if (Math.abs(turningAngle) > 5){ //robot is not parallel by more than 5 degree, adjust
            wheels.turnWithGyro(Math.abs(turningAngle), .25, d, gyro, this);
            DbgLog.msg("[Phoenix] At beacon, performed Reached beacon turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
        }

        imageX = MyUtility.getImageXPosition(tracker.get(1));
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
            adjustmentDistance = adjustmentDistance + 1; //need to move forward a bit more to handle the strafing problem

        if (adjustDirection == Direction.BACKWARD)
            DbgLog.msg("[Phoenix:Beacon Distance Adjustment 2] 2nd Beacon X position adjustment backward %7.3f and image X %7.3f", adjustmentDistance, imageX);
        else
            DbgLog.msg("[Phoenix:Beacon Distance Adjustment 2] 2nd Beacon X position adjustment forward %7.3f and imageX %7.3f", adjustmentDistance, imageX);

        if(adjustmentDistance >= 1) {
            DbgLog.msg("[Phoenix:Beacon Distance Adjustment 2] Performed 2nd Beacon X position adjustment %7.3f", adjustmentDistance);
            wheels.drive((int) adjustmentDistance, adjustDirection, 0.25, 5, this);
        }

        boolean pushAnyway = false;  //if see blue on one side, then, the other side got to be red
        if(color.red() <= 1) {
            DbgLog.msg("[Phoenix:Beacon Distance Adjustment 2] Can't see red, move back 6 inches");

            if (color.blue() > 1)
                pushAnyway = true; //if see blue on one side, then, the other side got to be red

            wheels.drive(7, Direction.BACKWARD, 0.25, 5, this);
        }

        if ((color.red() > 1) || pushAnyway) { //sees the red side or the other side is blue
            pusher.setPosition(0);
            Thread.sleep(1000);
            wheels.strafe(2, .8, 2, TurnDirection.LEFT, this);
            Thread.sleep(200);

            pusher.setPosition(1);
            wheels.strafe(10, .8, 5, TurnDirection.RIGHT, this);
        }
        else {
            wheels.strafe(8, .8, TurnDirection.RIGHT, this);
        }

        //Adjust before going for the center vortex
        endHeading = gyro.getIntegratedZValue();
        turningAngle = heading - endHeading;

        if(turningAngle < 0)
            d = TurnDirection.RIGHT;
        else
            d = TurnDirection.LEFT;

        if (d == TurnDirection.LEFT)
            DbgLog.msg("[Phoenix:Adjust before going for center Vortex] Leaving first beacon, Reached beacon turn LEFT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
        else
            DbgLog.msg("[PhoenixPhoenix:Adjust before going for center Vortex] Leaving first beacon, Reached beacon turn RIGHT, turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);

        if (Math.abs(turningAngle) > 5){
            wheels.turnWithGyro(Math.abs(turningAngle), .25, d, gyro, this);
            DbgLog.msg("[PhoenixPhoenix:Adjust before going for center Vortex] Leaving first beacon, performed Reached beacon turningAngle= %d, heading= %d endHeading=%d", turningAngle, heading, endHeading);
        }
        //Go for center vortex
        wheels.turnWithGyro(40, .5, TurnDirection.LEFT, gyro, this);
        pusher.setPosition(0.5);
        wheels.drive(55, Direction.BACKWARD, .8, 6, this);
    }

    void pushBeacon(Direction d, double ms) throws InterruptedException {
        int direction;

        if(d == Direction.FORWARD)
            direction = 0;
        else
            direction = 1;

        double wantedTime = this.getRuntime() + ms; //determine when this should end
        while((this.getRuntime() < wantedTime) && opModeIsActive()) {
            pusher.setPosition(direction);
        }

        pusher.setPosition(.5);
    }
}
