package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;

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

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        wheels = new FlamingPhoenix.MecanumDriveTrain("frontleft", "frontright", "backleft", "backright", "leftwheels", "rightwheels", this);

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
        int priorTurnedAngle = -1;
        int turnedAngle = 0;

        //wheels.turnAjdustment(0.25, TurnDirection.LEFT, this);
        wheels.turnWithGyro(88, 0.3, TurnDirection.LEFT, gyro, this);

        for(int i=0; i<500; i++) {
            int currentHeading = gyro.getIntegratedZValue();
            turnedAngle = Math.abs(currentHeading - startHeading);
            long changedTime = System.currentTimeMillis() - startTime;

            DbgLog.msg("[Phoenix:TurnComplete] power=%7.2f; time=%d; angle=%d", 0.0f, changedTime, turnedAngle);

            if (turnedAngle != priorTurnedAngle) {

            }

            priorTurnedAngle = turnedAngle;

            this.idle();
        }
    }
}
