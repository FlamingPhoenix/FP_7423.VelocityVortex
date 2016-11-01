package FlamingPhoenix;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

//Chassis
@Disabled
public class MecanumDriveTrain {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private OpMode opMode;

    private ModernRoboticsI2cGyro gyro;

    private float wheelDiameter;  //wheel diameter in inch
    private int encoderPPR; //wheel encoder PPR (Pulse per Rotation)

    public MecanumDriveTrain(String FrontLeftName, String FrontRightName, String BackLeftName, String BackRightName, ModernRoboticsI2cGyro gyroscope, OpMode OperatorMode) {
        opMode = OperatorMode;

        frontLeft = opMode.hardwareMap.dcMotor.get(FrontLeftName);
        frontRight = opMode.hardwareMap.dcMotor.get(FrontRightName);
        backLeft = opMode.hardwareMap.dcMotor.get(BackLeftName);
        backRight = opMode.hardwareMap.dcMotor.get(BackRightName);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro = gyroscope;

        wheelDiameter = 4; //default is 4 inch, most of wheels we have used are 4 inch diamete;
        encoderPPR = 1320; //default to Tetrix encoder;  AndyMark will have different values
    }

    //
    //
    //
    //TeleOp Methods

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

        while (backLeft.getCurrentPosition() < pulseNeeded) {
            opMode.telemetry.addData("Encoder: ", backLeft.getCurrentPosition());
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);
        }
    }

    public void Drive() {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while (backLeft.getCurrentPosition() < 1051) {
            frontLeft.setPower(75);
            frontRight.setPower(75);
            backRight.setPower(75);
            backLeft.setPower(75);
        }
    }

    private void mecanumDrive(float x1, float y1, float x2) {
        x1 = (float) scaleInput(x1);
        y1 = (float) scaleInput(y1);

        float FL = y1 + x1 + x2;
        float FR = y1 - x1 - x2;
        float BL = y1 - x1 + x2;
        float BR = y1 + x1 - x2;

        float mv = max(Math.abs(FL), Math.abs(FR), Math.abs(BL), Math.abs(BR));
        if (Math.abs(mv) > 1) {
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


    private int getOffsetToTarget(int startHeading, int newHeading, TurnDirection direction, ModernRoboticsI2cGyro gyro, OpMode OpModeInstance) {
        int currentHeading = gyro.getHeading();
        OpModeInstance.telemetry.addData("current Heading ", currentHeading);
        OpModeInstance.telemetry.addData("startHeading: ", startHeading);
        OpModeInstance.telemetry.addData("newHeading: ", newHeading);

        DbgLog.msg("[Phoenix] currentHeading =  " + Integer.toString(currentHeading));
        DbgLog.msg("[Phoenix] startHeading =  " + Integer.toString(startHeading));
        DbgLog.msg("[Phoenix] newHeading =  " + Integer.toString(newHeading));

        if ((direction == TurnDirection.RIGHT) || direction == TurnDirection.RIGHT_FORWARD) {
            if ((currentHeading < startHeading) && ((currentHeading >= 0) && (currentHeading < 180) && (startHeading >= 180))) {
                currentHeading += 360;
            } else if ((Math.abs(currentHeading - startHeading) == 1) || ((currentHeading == 359) && (startHeading == 0)))
                currentHeading = startHeading;

            return newHeading - currentHeading;
        } else {
            if ((currentHeading > startHeading) && ((currentHeading >= 180) && (currentHeading < 360) && (startHeading >= 0) && (startHeading < 180))) {
                currentHeading -= 360;
            } else if ((Math.abs(currentHeading - startHeading) == 1) || ((currentHeading == 0) && (startHeading == 359))) {
                currentHeading = startHeading;
            }

            return currentHeading - newHeading;
        }
    }

    //Use ModernRobotic Gyroscope integrated Z heading to control turning
    public void turnWithGyro(int degree, double power, TurnDirection direction, boolean allWheels, ModernRoboticsI2cGyro gyro, LinearOpMode opMode) throws InterruptedException {
        while (gyro.isCalibrating())
            Thread.sleep(50);
        gyro.resetZAxisIntegrator(); //reset z heading to zero
        opMode.idle();

        int startHeading = gyro.getIntegratedZValue();
        int targetHeading = startHeading + (direction == TurnDirection.RIGHT ? degree * -1 : degree);

        DbgLog.msg("[Phoenix] targetHeading = " + targetHeading);

        double speed = Math.abs(power);
        if (direction == TurnDirection.LEFT)
            speed = speed  * -1;


        long time1 = System.currentTimeMillis(); //Use this to measure the time internal (start time)
        int heading1 = startHeading; //use this to measure the degree/speed difference

        int currentHeading = gyro.getIntegratedZValue();
        DbgLog.msg("[Phoenix] currentHeading = " + currentHeading);


        while (opMode.opModeIsActive()) {
            int targetMargin = 0;

            //calculate the speed to decide how far ahead we need to reduce the power to 0
            int headingDifference = currentHeading - heading1;
            if (headingDifference >= 2) { //Robot has turned more than 2 degrees, let's figure out the turning speed
                long time2 = System.currentTimeMillis();
                long turningSpeed = ((long) headingDifference * 1000L) / (time2 - time1);

                DbgLog.msg("[Phoenix:s] turningSpeed = " + Long.toString(turningSpeed));

                time1 = time2;
                heading1 = currentHeading; //reset the starting value for next interval

                targetMargin = (int) turningSpeed / 10;
                DbgLog.msg("[Phoenix:s] targetMargin = " + Integer.toString(targetMargin));
            }

            DbgLog.msg("[Phoenix:s] currentHeading = " + Integer.toString(currentHeading));
            if(Math.abs(currentHeading - targetHeading) < targetMargin)
                break;
            else if (Math.abs(currentHeading - targetHeading) < 15)
                speed = speed * 0.7;

            frontRight.setPower(speed * -1);
            backRight.setPower(speed * -1);
            frontLeft.setPower(speed);
            backLeft.setPower(speed);

            opMode.idle();
            currentHeading = gyro.getIntegratedZValue();
            DbgLog.msg("[Phoenix] currentHeading = " + currentHeading);
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        DbgLog.msg("[Phoenix:s] final1currentHeading = " + Double.toString(gyro.getIntegratedZValue()));
        Thread.sleep(2000);
        DbgLog.msg("[Phoenix:s] final2currentHeading = " + Double.toString(gyro.getIntegratedZValue()));
    }

    private double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

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

        for (int i = 0; i < args.length; i++) {
            if (args[i] > m)
                m = args[i];
        }

        return m;
    }
}

