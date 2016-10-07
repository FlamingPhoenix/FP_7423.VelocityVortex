package org.firstinspires.ftc.robotcontroller.FlamingPhoenix;

import android.support.annotation.StringDef;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

//Chassis
public class MecanumDriveTrain {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private OpMode opMode;

    private float wheelDiameter;  //wheel diameter in inch
    private int encoderPPR; //wheel encoder PPR (Pulse per Rotation)

    public MecanumDriveTrain(String FrontLeftName, String FrontRightName, String BackLeftName, String BackRightName, OpMode OperatorMode) {
        opMode = OperatorMode;

        frontLeft = opMode.hardwareMap.dcMotor.get(FrontLeftName);
        frontRight = opMode.hardwareMap.dcMotor.get(FrontRightName);
        backLeft = opMode.hardwareMap.dcMotor.get(BackLeftName);
        backRight = opMode.hardwareMap.dcMotor.get(BackRightName);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelDiameter = 4; //default is 4 inch, most of wheels we have used are 4 inch diamete;
        encoderPPR = 1320; //default to Tetrix encoder;  AndyMark will have different values
    }

    /*
     * Initilize the MecanumDriveTrain to set the value of WheelDiameter and EncoderPPR
     * @param WheelDiameter Drive Wheel's diameter in inch
     * @param EncoderPPR  Encoder's Pulse per Rotation
     */
    public void init(float WheelDiameter, int EncoderPPR) {
        wheelDiameter = WheelDiameter;
        encoderPPR = EncoderPPR;
    }

    /*
     * Control or drive the robot using gamepad
     * @param gamePad
     */
    public void Drive(Gamepad gamePad) {
        mecanumDrive(-gamePad.left_stick_x, gamePad.left_stick_y, gamePad.right_stick_x);
    }

   public void Drive(int d, int power, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int pulseNeeded = (int) Math.round((encoderPPR * d) / (wheelDiameter * Math.PI));

        while(backLeft.getCurrentPosition() < pulseNeeded) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);
        }
    }

    public void Drive() {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while(backLeft.getCurrentPosition() < 1051) {
            frontLeft.setPower(75);
            frontRight.setPower(75);
            backRight.setPower(75);
            backLeft.setPower(75);
        }
    }

    private void mecanumDrive(float x1, float y1, float x2) {
        x1 = (float)scaleInput(x1);
        y1 =  (float)scaleInput(y1);

        float FL =  y1 + x1 + x2;
        float FR =  y1 - x1 - x2;
        float BL =  y1 - x1 + x2;
        float BR =  y1 + x1 - x2;

        float mv = max(Math.abs(FL), Math.abs(FR), Math.abs(BL), Math.abs(BR));
        if (mv > 0) {
            FL = FL / mv;
            FR = FR / mv;
            BL = BL / mv;
            BR = BR / mv;
        }

        FL = Range.clip(FL, -1, 1);
        FR = Range.clip(FR, -1, 1);
        BL = Range.clip(BL, -1, 1);
        BR = Range.clip(BR, -1, 1);

        frontLeft.setPower(FL);
        frontRight.setPower(FR);
        backLeft.setPower(BL);
        backRight.setPower(BR);
    }

    public void turnUsingGyro(int degree, double power, TurnDirection direction, boolean bothWheels, GyroSensor gyro, LinearOpMode opModeInstance) throws InterruptedException {
        while (gyro.isCalibrating())
            Thread.sleep(50);

        int startHeading = gyro.getHeading();
        int newHeading = startHeading + (direction == TurnDirection.RIGHT ? degree : direction == TurnDirection.RIGHT_FORWARD ? degree : degree * -1);
        opModeInstance.telemetry.addData("startHeading: ", startHeading);
        opModeInstance.telemetry.addData("newHeading: ", newHeading);

        double speed = Math.abs(power);

        int targetOffset = getOffsetToTarget(startHeading, newHeading, direction, gyro, opModeInstance);
        int previousOffset = targetOffset;
        while (targetOffset > 2) {
            int degreeChanged = Math.abs(targetOffset - previousOffset);
            if (degreeChanged >= targetOffset)
                speed = 0;
            else if (targetOffset == previousOffset) {
                if (speed == 0)
                    speed = 0.05;
                double speedAdjustment = speed * 1.1;
                if (speedAdjustment <= power)
                    speed = speedAdjustment;
                else
                    speed = Math.abs(power);
            } else if (targetOffset <= 10)
                speed = 0.005;


            if ((direction == TurnDirection.RIGHT) || (direction == TurnDirection.RIGHT_FORWARD)) {
                if (bothWheels) {
                    frontRight.setPower(speed);
                    backRight.setPower(speed);
                    frontRight.setPower(speed * -1);
                    backLeft.setPower(speed * -1);
                }
                else if (direction == TurnDirection.RIGHT_FORWARD) {
                    frontRight.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(speed * -1);
                    backLeft.setPower(speed * -1);
                }
                else {
                    frontRight.setPower(speed);
                    backRight.setPower(speed);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                }
            } else {
                if (bothWheels) {
                    frontRight.setPower(speed * -1);
                    backRight.setPower(speed * -1);
                    frontRight.setPower(speed);
                    backLeft.setPower(speed);
                }
                else if (direction == TurnDirection.LEFT_FORWARD) {
                    frontRight.setPower(speed * -1);
                    backRight.setPower(speed * -1);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                }
                else {
                    frontRight.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(speed);
                    backLeft.setPower(speed);
                }
            }
            previousOffset = targetOffset;
            opModeInstance.waitForNextHardwareCycle();
            targetOffset = getOffsetToTarget(startHeading, newHeading, direction, gyro, opModeInstance);
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        opModeInstance.waitOneFullHardwareCycle();

        opModeInstance.telemetry.addData("startHeading: ", startHeading);
        opModeInstance.telemetry.addData("newHeading: ", newHeading);
        int finalHeading = gyro.getHeading();
        opModeInstance.telemetry.addData("Final Heading", finalHeading);
        //DbgLog.msg("[Phoenix] turnUsingGyro finalHeading: " + Integer.toString(finalHeading));
    }

    private int getOffsetToTarget(int startHeading, int newHeading, TurnDirection direction, GyroSensor gyro, OpMode OpModeInstance) {
        int currentHeading = gyro.getHeading();
        OpModeInstance.telemetry.addData("current Heading ", currentHeading);
        OpModeInstance.telemetry.addData("startHeading: ", startHeading);
        OpModeInstance.telemetry.addData("newHeading: ", newHeading);

        if( (direction == TurnDirection.RIGHT) || direction == TurnDirection.RIGHT_FORWARD) {
            if( (currentHeading < startHeading) && ((currentHeading >= 0) && (currentHeading < 180) && (startHeading >= 180))) {
                currentHeading += 360;
            }
            else if ((Math.abs(currentHeading - startHeading) == 1) || ((currentHeading == 359) && (startHeading == 0)))
                currentHeading = startHeading;

            return newHeading - currentHeading;
        }
        else {
            if ((currentHeading > startHeading) && ((currentHeading >= 180) && (currentHeading < 360) && (startHeading >=0) && (startHeading < 180))) {
                currentHeading -= 360;
            }
            else if ((Math.abs(currentHeading - startHeading) == 1) || ((currentHeading == 0) && (startHeading == 359))) {
                currentHeading = startHeading;
            }

            return currentHeading - newHeading;
        }
    }

    private double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

    private float max(float... args) {
        float m = 0;

        for (int i=0; i<args.length; i++ ) {
            if (args[i] > m)
                m = args[i];
        }

        return m;
    }
}
