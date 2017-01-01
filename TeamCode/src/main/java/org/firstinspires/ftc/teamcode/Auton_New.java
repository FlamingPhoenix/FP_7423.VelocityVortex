package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import FlamingPhoenix.Direction;
import FlamingPhoenix.MecanumDriveTrain;
import FlamingPhoenix.TurnDirection;
import FlamingPhoenix.MyUtility;
/**
 * Created by HwaA1 on 11/26/2016.
 */
@Autonomous(name = "Turn_Test", group = "none")
public class Auton_New extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    VuforiaTrackables tracker;

    DcMotor shooter;

    Servo stopper;

    ColorSensor color;

    OpticalDistanceSensor opt;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.dcMotor.get("farriswheel");
        stopper = hardwareMap.servo.get("stopper");

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

        opt = hardwareMap.opticalDistanceSensor.get("opt");

        while (gyro.isCalibrating() && this.opModeIsActive())
            Thread.sleep(50);

        MecanumDriveTrain wheels = new MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);

        //shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //this.idle();
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //shooter.setMaxSpeed(960);

        gyro.resetZAxisIntegrator();
        gyro.calibrate();

        while(gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        waitForStart();

        shooter.setPower(.7);

        wheels.strafe(5, 1500, TurnDirection.LEFT, this);
        Thread.sleep(1000);

        stopper.setPosition(.25);
        Thread.sleep(250);
        stopper.setPosition(.75);
        Thread.sleep(500);
        stopper.setPosition(0.25);
        Thread.sleep(1000);
        stopper.setPosition(.75);

        wheels.strafe(20, 1200, TurnDirection.LEFT, gyro, this);

        shooter.setPower(0);

        wheels.drive(35, Direction.BACKWARD, 1400, this);

        int angleBefore= gyro.getIntegratedZValue();
        int degreesNeeded = Math.abs(90 - angleBefore);
        wheels.turnWithGyro(degreesNeeded, .25, TurnDirection.LEFT, gyro, this);
        int angleAfter = gyro.getIntegratedZValue();

        DbgLog.msg("[Phoenix] angleBefore= %d, degreeNeeded= %d, angleAfter= %d", angleBefore, degreesNeeded, angleAfter);

        wheels.drive(11, Direction.FORWARD, 1500, this);
        Thread.sleep(250);

        wheels.driveUntilImage(20, .1, Direction.FORWARD, tracker.get(3), this);
        int angle = MyUtility.getImageAngle(tracker.get(3));
        DbgLog.msg("[Phoenix] angle1: " + angle);

        Thread.sleep(250);
        angle = MyUtility.getImageAngle(tracker.get(3));

        DbgLog.msg("[Phoenix] angle2: " + angle);

        int turnAngle = Math.abs(90 - angle);

        if(angle < 90) {
            DbgLog.msg("[Phoeix] Adjust to turn left by %d degree based on image angle of %d", turnAngle, angle);
            wheels.turnWithGyro(turnAngle, .5, TurnDirection.RIGHT, gyro, this);
        }
        else if(angle > 90) {
            DbgLog.msg("[Phoenix] Adjust to turn right by %d degree based on image angle of %d", turnAngle, angle);
            wheels.turnWithGyro(turnAngle, .5, TurnDirection.LEFT, gyro, this);
        }

        int heading = gyro.getIntegratedZValue();
        wheels.resetMotorSpeed();
        wheels.strafe(200, 0.8, TurnDirection.LEFT, tracker.get(3), this);

        int endHeading = gyro.getIntegratedZValue();
        int turningAngle = heading - endHeading;
        TurnDirection d;

        if(turningAngle < 0)
            d = TurnDirection.RIGHT;
        else
            d = TurnDirection.LEFT;

        wheels.turnWithGyro(Math.abs(turningAngle), .3, d, gyro, this);
        DbgLog.msg("[Phoenix] Reached beacon turnAngle= %d, heading= %d endHeading=%d", turnAngle, heading, endHeading);

        float imageX = MyUtility.getImageXPosition(tracker.get(3));
        DbgLog.msg("[Phoenix] imageX=%9.3f", imageX);

        float adjustmentDistance;
        Direction adjustDirection;
        if(color.red() <= 1) {
            if ((imageX > 0) && imageX < 76) {// We need to move back (to the left of image a bit)
                adjustDirection = Direction.BACKWARD;
                adjustmentDistance = 76 - imageX;
            }
            else if ((imageX > 0) && (imageX > 76)) {// We need to move forward (to the right a bit)
                adjustDirection = Direction.FORWARD;
                adjustmentDistance = 76 + imageX;
            }
            else if (imageX < 0 && imageX > -76) {// We need to move forward
                adjustDirection = Direction.FORWARD;
                adjustmentDistance = 76 + imageX;
            }
            else if (imageX < 0 && imageX < -76) {// We need to move back a bit
                adjustDirection = Direction.BACKWARD;
                adjustmentDistance = 76 - imageX;
            }
            else {
                adjustDirection = Direction.FORWARD;
                adjustmentDistance = 0;
            }

            adjustmentDistance /= 25.4;
            adjustmentDistance = Math.round(adjustmentDistance);
            if(adjustmentDistance > 1) {
                wheels.drive((int) adjustmentDistance, adjustDirection, 1300, this);
            }
        }

        wheels.strafe(5, .5, TurnDirection.LEFT, this);
        Thread.sleep(500);
        wheels.strafe(5, .5, TurnDirection.RIGHT, this);

    }

}
