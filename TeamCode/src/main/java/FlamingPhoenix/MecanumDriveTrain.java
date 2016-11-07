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
        //encoderPPR = 1320; //default to Tetrix encoder;  AndyMark will have different values
        encoderPPR = 560;
    }

    public MecanumDriveTrain(String FrontLeftName, String FrontRightName, String BackLeftName, String BackRightName, OpMode OperatorMode) {
        opMode = OperatorMode;

        frontLeft = opMode.hardwareMap.dcMotor.get(FrontLeftName);
        frontRight = opMode.hardwareMap.dcMotor.get(FrontRightName);
        backLeft = opMode.hardwareMap.dcMotor.get(BackLeftName);
        backRight = opMode.hardwareMap.dcMotor.get(BackRightName);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelDiameter = 4; //default is 4 inch, most of wheels we have used are 4 inch diamete;
        //encoderPPR = 1320; //default to Tetrix encoder;  AndyMark will have different values
        encoderPPR = 560;
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


    private void mecanumDrive(float x1, float y1, float x2) {
        x1 = (float) scaleInput(x1);
        y1 = (float) scaleInput(y1) * -1;

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


    //
    //
    //
    //Autonomous methods

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

    public void turnUsingGyro(int degree, double power, TurnDirection direction, boolean bothWheels, ModernRoboticsI2cGyro gyro, LinearOpMode opModeInstance) throws InterruptedException {
        while (gyro.isCalibrating())
            Thread.sleep(50);

        int startHeading = gyro.getHeading();
        int newHeading = startHeading + (direction == TurnDirection.RIGHT ? degree : direction == TurnDirection.RIGHT_FORWARD ? degree : degree * -1);
        opModeInstance.telemetry.addData("startHeading: ", startHeading);
        opModeInstance.telemetry.addData("newHeading: ", newHeading);

        double speed = Math.abs(power);

        int targetOffset = getOffsetToTarget(startHeading, newHeading, direction, gyro, opModeInstance);
        DbgLog.msg("[Phoenix] targetOffset =  " + Integer.toString(targetOffset));
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
                    frontLeft.setPower(speed * -1);
                    backLeft.setPower(speed * -1);
                } else if (direction == TurnDirection.RIGHT_FORWARD) {
                    frontRight.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(speed * -1);
                    backLeft.setPower(speed * -1);
                } else {
                    frontRight.setPower(speed);
                    backRight.setPower(speed);
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                }
            } else {
                if (bothWheels) {
                    frontRight.setPower(speed * -1);
                    backRight.setPower(speed * -1);
                    frontLeft.setPower(speed);
                    backLeft.setPower(speed);
                } else if (direction == TurnDirection.LEFT_FORWARD) {
                    frontRight.setPower(speed * -1);
                    backRight.setPower(speed * -1);
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                } else {
                    frontRight.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(speed);
                    backLeft.setPower(speed);
                }
            }
            previousOffset = targetOffset;
            opModeInstance.idle();
            targetOffset = getOffsetToTarget(startHeading, newHeading, direction, gyro, opModeInstance);
            DbgLog.msg("[Phoenix] targetOffset =  " + Integer.toString(targetOffset));
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        opModeInstance.idle();

        opModeInstance.telemetry.addData("startHeading: ", startHeading);
        opModeInstance.telemetry.addData("newHeading: ", newHeading);
        int finalHeading = gyro.getHeading();
        opModeInstance.telemetry.addData("Final Heading", finalHeading);
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

    public void turnWithGyro(int degree, double power, TurnDirection direction, boolean allWheels, ModernRoboticsI2cGyro gyro, LinearOpMode opMode) throws InterruptedException {
        while (gyro.isCalibrating())
            Thread.sleep(50);

        gyro.resetZAxisIntegrator();

        int startHeading = gyro.getIntegratedZValue();
        int targetHeading = startHeading + (direction == TurnDirection.RIGHT ? degree * -1 : degree);

        DbgLog.msg("[Phoenix] targetHeading = " + targetHeading);

        double speed = Math.abs(power);

        int currentHeading = gyro.getIntegratedZValue();
        DbgLog.msg("[Phoenix] currentHeading = " + currentHeading);

        if(targetHeading < 0) { //negative means turning right and vice versa
            while((currentHeading > targetHeading) && (opMode.opModeIsActive())) {
                if(Math.abs(currentHeading - targetHeading) < 7.5)
                    break;
                frontRight.setPower(speed * -1);
                backRight.setPower(speed * -1);
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                currentHeading = gyro.getIntegratedZValue();

                DbgLog.msg("[Phoenix] currentHeading = " + currentHeading);
            }
        } else {
            while((currentHeading < targetHeading) && (opMode.opModeIsActive())){
                if(Math.abs(targetHeading - currentHeading) < 7.5)
                    break;
                frontRight.setPower(speed);
                backRight.setPower(speed);
                frontLeft.setPower(speed * -1);
                backLeft.setPower(speed * -1);
                currentHeading = gyro.getIntegratedZValue();

                DbgLog.msg("[Phoenix] currentHeading = " + currentHeading);
            }
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        DbgLog.msg("[Phoenix] final1currentHeading = " + Double.toString(gyro.getIntegratedZValue()));
        Thread.sleep(1000);
        DbgLog.msg("[Phoenix] final2currentHeading = " + Double.toString(gyro.getIntegratedZValue()));
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

    public void strafe(int distance, double power, TurnDirection direction, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int pulseNeeded = (int) Math.round((encoderPPR * distance) / (wheelDiameter * Math.PI));

        pulseNeeded *= .45; //the distance is around .45 the normal

        while((frontRight.getCurrentPosition() < pulseNeeded) && opMode.opModeIsActive()) {
            if (direction == TurnDirection.LEFT) {
                frontLeft.setPower(power * -1);
                backLeft.setPower(power);
                frontRight.setPower(power);
                backRight.setPower(power * -1);
            } else {
                frontLeft.setPower(power);
                backLeft.setPower(power * -1);
                frontRight.setPower(power * -1);
                backRight.setPower(power);
            }
        }

        frontLeft.setPower(0  );
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
}

