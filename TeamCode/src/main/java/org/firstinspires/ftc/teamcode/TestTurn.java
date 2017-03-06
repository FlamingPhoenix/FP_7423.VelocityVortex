package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import FlamingPhoenix.*;
import FlamingPhoenix.MecanumDriveTrain;
import FlamingPhoenix.TurnDirection;

/**
 * Created by Steve on 3/5/2017.
 */

@Autonomous(name = "Test Turn", group = "Practice")
public class TestTurn extends LinearOpMode {

    MecanumDriveTrain wheels;
    ModernRoboticsI2cGyro gyro;

    ///Initialize the routine
    private void initialize() throws InterruptedException {

        wheels = new FlamingPhoenix.MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", this);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        gyro.resetZAxisIntegrator();
        gyro.resetZAxisIntegrator();
        gyro.calibrate();

        while(gyro.isCalibrating()) {
            Thread.sleep(50);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        this.waitForStart();

        int startHeading = gyro.getIntegratedZValue();
        long startTime = System.currentTimeMillis();
        int priorChangedAngle = -1;

        //wheels.turnAjdustment(0.25, TurnDirection.LEFT, this);
        wheels.turnWithGyro(88, 0.25, TurnDirection.LEFT, gyro, this);

        for(int i=0; i<1000; i++) {
            int currentHeading = gyro.getIntegratedZValue();
            int changedAngle = Math.abs(currentHeading - startHeading);
            long changedTime = System.currentTimeMillis() - startTime;

            DbgLog.msg("[Phoenix:TurnComplete] power=%7.2f; time=%d; angle=%d", 0.0f, changedTime, changedAngle);

            if (changedAngle != priorChangedAngle) {

                priorChangedAngle = changedAngle;
            }

            this.idle();
        }
    }
}
