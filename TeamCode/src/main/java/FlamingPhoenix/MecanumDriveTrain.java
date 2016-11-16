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
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

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
    private ElapsedTime runtime = new ElapsedTime();

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

        float x1;
        float y1;
        float x2;

        if ((gamePad.right_trigger > 0.5) && (gamePad.left_trigger > 0.5))  //both trigger are pressed, let's slow down a lot more
        {
            x1 = gamePad.left_stick_x * 0.25f;
            y1 = gamePad.left_stick_y * 0.25f;
            x2 = gamePad.right_stick_x * 0.25f;
        } else if ((gamePad.right_trigger > 0.5) || (gamePad.left_trigger > 0.5)) { //only one of the trigger is pressed, let's slow down by 50%
            x1 = gamePad.left_stick_x * 0.5f;
            y1 = gamePad.left_stick_y * 0.5f;
            x2 = gamePad.right_stick_x * 0.5f;
        }
        else {
            x1 = gamePad.left_stick_x;
            y1 = gamePad.left_stick_y;
            x2 = gamePad.right_stick_x;
        }

        mecanumDrive(x1, y1, x2);
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

    public void drive(int d, double power, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int pulseNeeded = (int) Math.round((encoderPPR * d) / (wheelDiameter * Math.PI));

        while ((Math.abs(backLeft.getCurrentPosition()) < pulseNeeded) && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Encoder: ", backLeft.getCurrentPosition());
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);
            DbgLog.msg("[Phoenix] Back Left encoder: " + backLeft.getCurrentPosition());
            DbgLog.msg("[Phoenix] Back Right encoder: " + backRight.getCurrentPosition());
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    //Drive by using PIDControl - Run to Position
    public void drive(int d, Direction direction, double power, int timeout, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int pulseNeeded = (int) Math.round((encoderPPR * d) / (wheelDiameter * Math.PI));
        if (direction == Direction.BACKWARD)
            pulseNeeded = pulseNeeded * -1;

        backLeft.setTargetPosition(pulseNeeded);
        backRight.setTargetPosition(pulseNeeded);
        frontLeft.setTargetPosition(pulseNeeded);
        frontRight.setTargetPosition(pulseNeeded);

        DbgLog.msg("[Phoenix] pulseNeeded: " + pulseNeeded);

        power = Math.abs(power);

        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);

        runtime.reset();
        while ((opMode.opModeIsActive()) && (runtime.seconds() <= timeout) && (backLeft.isBusy()) && (backRight.isBusy()) && (frontLeft.isBusy()) && (frontRight.isBusy())) {
            opMode.telemetry.addData("running", "");
            opMode.telemetry.update();
        }

        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

        DbgLog.msg("[Phoenix] backLeft" + backLeft.getCurrentPosition());
        DbgLog.msg("[Phoenix] backRight" + backRight.getCurrentPosition());
        DbgLog.msg("[Phoenix] frontLeft" + frontLeft.getCurrentPosition());
        DbgLog.msg("[Phoenix] frontRight" + frontRight.getCurrentPosition());

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Drive by controlling the motor rotation speed using encoder
    public void drive(int d, Direction direction, int speed, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMaxSpeed(speed);
        backRight.setMaxSpeed(speed);
        frontLeft.setMaxSpeed(speed);
        frontRight.setMaxSpeed(speed);

        int pulseNeeded = (int) Math.round((encoderPPR * d) / (wheelDiameter * Math.PI));
        double power = 1;
        if (direction == Direction.BACKWARD)
            power = -1;

        while ((Math.abs(backLeft.getCurrentPosition()) < pulseNeeded) && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Encoder: ", backLeft.getCurrentPosition());
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);
            DbgLog.msg("[Phoenix] Back Left encoder: " + backLeft.getCurrentPosition());
            DbgLog.msg("[Phoenix] Back Right encoder: " + backRight.getCurrentPosition());
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
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

    //get the maximum value among the values in the input
    private float max(float... args) {
        float m = 0;

        for (int i = 0; i < args.length; i++) {
            if (args[i] > m)
                m = args[i];
        }

        return m;
    }
    //get the maximum value among the values in the input
    private double max(double... args) {
        double m = 0;

        for (int i = 0; i < args.length; i++) {
            if (args[i] > m)
                m = args[i];
        }

        return m;
    }
    //get the maximum value among the values in the input
    private int max(int... args) {
        int m = 0;

        for (int i = 0; i < args.length; i++) {
            if (args[i] > m)
                m = args[i];
        }

        return m;
    }

    //strafe by setting power
    public void strafe(int distance, double power, TurnDirection direction, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int pulseNeeded = (int) Math.round((encoderPPR * distance) / (wheelDiameter * Math.PI));

        pulseNeeded = (int) Math.round((double) pulseNeeded/0.5); //the distance is around .65 the normal

        while(((Math.abs(backLeft.getCurrentPosition())) < pulseNeeded) && opMode.opModeIsActive()) {
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

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    //strafe by using the wheel rotation speed
    public void strafe(int distance, int speed, TurnDirection direction, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMaxSpeed(speed);
        backRight.setMaxSpeed(speed);
        frontLeft.setMaxSpeed(speed);
        frontRight.setMaxSpeed(speed);

        int pulseNeeded = (int) Math.round((encoderPPR * distance) / (wheelDiameter * Math.PI));

        pulseNeeded = (int) Math.round((double) pulseNeeded/0.5); //the distance is around .65 the normal

        while(((Math.abs(backLeft.getCurrentPosition())) < pulseNeeded) && opMode.opModeIsActive()) {
            if (direction == TurnDirection.LEFT) {
                frontLeft.setPower(-1);
                backLeft.setPower(1);
                frontRight.setPower(1);
                backRight.setPower(-1);
            } else {
                frontLeft.setPower(1);
                backLeft.setPower(-1);
                frontRight.setPower(-1);
                backRight.setPower(1);
            }
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }


    public void strafe(int distance, double power, TurnDirection direction, ModernRoboticsI2cGyro gyroscope,LinearOpMode opMode) throws InterruptedException {
        int startingDirection = gyroscope.getIntegratedZValue();
        DbgLog.msg("[Phoenix] startingDirection: " + startingDirection);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int pulseNeeded = (int) Math.round((encoderPPR * distance) / (wheelDiameter * Math.PI));

        pulseNeeded /= .5; //the distance is around .45 the normal

        while(opMode.opModeIsActive() && Math.abs(backLeft.getCurrentPosition()) < pulseNeeded) {
            int currentDirection = gyroscope.getIntegratedZValue(); //currentDirection will be updated in the while loop to track the robot's direction.
            int changeInDirection = currentDirection - startingDirection;  //track how much direction has changed. Positive value means going left, negative means going to the right

            double frontLeftPower = power; //we need to determine if we need to adjust the left front power to adjust for direction to the right
            double backLeftPower = power; //this is to increase the back left if we need to go left
            double frontRightPower = power; //need to increase this power if need to move the left
            double backRightPower = power; //need to increase this power if need to move the right

            double adjustmentUnit = power / 15;
            DbgLog.msg("[Phoenix] adjustmentUnit: " + adjustmentUnit);

            double powerAdjustment = 0;
            if (Math.abs(changeInDirection) > 1) //direction has change more than 2 degree, let's calculate how much power we need to adjust
                powerAdjustment = ((double) Math.abs(changeInDirection)) * adjustmentUnit;

            if (direction == TurnDirection.LEFT) {
                if (changeInDirection > 1) {//direction has moved to the left more than 2 degree, let's adjust
                    frontLeftPower = frontLeftPower + powerAdjustment * 4;
                    frontRightPower = frontRightPower + powerAdjustment * 4;
                }
                else if (changeInDirection < -1) { //direction has move to the right more than 2 degree
                    backLeftPower = backLeftPower + powerAdjustment;
                    backRightPower = backRightPower + powerAdjustment;
                }

                backLeftPower = backLeftPower * -1;  //strafing left, the power of back left is oposite of front left
                frontRightPower = frontRightPower * -1;  //strafing left, the power of the front Right is opposite of back right
            } else {
                if (changeInDirection > 1) { //direction has moved to the left more than 2 degree, let's adjust
                    backRightPower = backRightPower + powerAdjustment;
                    backLeftPower = backLeftPower + powerAdjustment;
                }
                else if (changeInDirection < -1) {//direction has move to the right more than 2 degree
                    frontRightPower = frontRightPower + powerAdjustment;
                    frontLeftPower = frontLeftPower + powerAdjustment;
                }

                frontLeftPower = frontLeftPower * -1;
                backRightPower = backRightPower * -1;
            }

            //if any of wheel is higher than 1.0, we need to reduce the power of all wheels proportionally.
            double mv = max(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backRightPower), Math.abs(backLeftPower));
            if (mv > 1) {
                frontLeftPower = frontLeftPower * Math.abs(frontLeftPower / mv);
                frontRightPower = frontRightPower * Math.abs(frontRightPower /mv);
                backLeftPower = backLeftPower * Math.abs(backLeftPower /mv);
                backRightPower = backRightPower * Math.abs(backRightPower /mv);
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            DbgLog.msg("[Phoenix] currentDirection: " + currentDirection);
            DbgLog.msg("[Phoenix] changeInDirection: " + changeInDirection);

            opMode.telemetry.addData("currentDirection ", currentDirection);
            opMode.telemetry.addData("changeInDirection ", changeInDirection);
            opMode.telemetry.update();

            opMode.idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    //control strafing using wheel rotation speed
    public void strafe(int distance, int speed, TurnDirection direction, ModernRoboticsI2cGyro gyroscope, LinearOpMode opMode) {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int startingDirection = gyroscope.getIntegratedZValue();
        DbgLog.msg("[Phoenix] startingDirection: " + startingDirection);

        int pulseNeeded = (int) Math.round((encoderPPR * distance) / (wheelDiameter * Math.PI));
        pulseNeeded /= .5; //the distance is around .5 the normal

        while(((Math.abs(backLeft.getCurrentPosition())) < pulseNeeded) && opMode.opModeIsActive()) {

            int currentDirection = gyroscope.getIntegratedZValue(); //currentDirection will be updated in the while loop to track the robot's direction.
            int changeInDirection = currentDirection - startingDirection;  //track how much direction has changed. Positive value means going left, negative means going to the right

            int frontLeftSpeed = Math.abs(speed); //we need to determine if we need to adjust the left front power to adjust for direction to the right
            int backLeftSpeed = Math.abs(speed); //this is to increase the back left if we need to go left
            int frontRightSpeed = Math.abs(speed); //need to increase this power if need to move the left
            int backRightSpeed = Math.abs(speed); //need to increase this power if need to move the right

            int adjustmentUnit = 14; //adjust 14 enocder ticks per second for every degree off course.
            int speedAdjustment = 0;
            if (Math.abs(changeInDirection) > 1) //direction has change more than 2 degree, let's calculate how much power we need to adjust
                speedAdjustment = Math.abs(changeInDirection) * adjustmentUnit;

            if (direction == TurnDirection.LEFT) {
                if (changeInDirection > 1) {//direction has moved to the left more than 1 degree, let's adjust
                    backLeftSpeed = backLeftSpeed + speedAdjustment;
                    backRightSpeed = backRightSpeed + speedAdjustment;
                }
                else if (changeInDirection < -1) { //direction has move to the right more than 1 degree
                    frontLeftSpeed = frontLeftSpeed + speedAdjustment;
                    frontRightSpeed = frontRightSpeed + speedAdjustment;
                }
            } else {
                if (changeInDirection > 1) { //direction has moved to the left more than 1 degree, let's adjust
                    frontRightSpeed = frontRightSpeed + speedAdjustment;
                    frontLeftSpeed = frontLeftSpeed + speedAdjustment;
                }
                else if (changeInDirection < -1) {//direction has move to the right more than 1 degree
                    backRightSpeed = backRightSpeed + speedAdjustment;
                    backLeftSpeed = backLeftSpeed + speedAdjustment;
                }
            }

            int mv = max(frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);
            if (mv > 2400) { //The max speed of Andymark Neverest 40 is 2400 encoder ticks per second.  RPM is 129 @ 1120 ticks per rotation
                frontLeftSpeed = frontLeftSpeed * frontLeftSpeed / mv;
                frontRightSpeed = frontRightSpeed * frontRightSpeed / mv;
                backLeftSpeed = backLeftSpeed * backLeftSpeed / mv;
                backRightSpeed = backRightSpeed * backRightSpeed / mv;
            }
            frontLeft.setMaxSpeed(frontLeftSpeed);
            backLeft.setMaxSpeed(backLeftSpeed);
            frontRight.setMaxSpeed(frontRightSpeed);
            backRight.setMaxSpeed(backRightSpeed);

            if (direction == TurnDirection.LEFT) {
                frontLeft.setPower(-1);
                backLeft.setPower(1);
                frontRight.setPower(1);
                backRight.setPower(-1);
            } else {
                frontLeft.setPower(1);
                backLeft.setPower(-1);
                frontRight.setPower(-1);
                backRight.setPower(1);
            }
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void strafe(int distance, double power, TurnDirection direction, VuforiaTrackableDefaultListener image, LinearOpMode opMode) throws InterruptedException {

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int pulseNeeded = (int) Math.round((encoderPPR * distance) / (wheelDiameter * Math.PI));

        pulseNeeded /= .65; //the distance is around .45 the normal
        double adjustmentUnit = power / 20;
        DbgLog.msg("[Phoenix] adjustmentUnit: " + adjustmentUnit);

        OpenGLMatrix pos = image.getPose();
        VectorF translation;
        double x = 0;
        double y = distance;

        while(opMode.opModeIsActive() && y >= (double) distance) {
            pos = image.getPose();

            if(pos != null) {
                translation = pos.getTranslation();

                x = translation.get(0) * -1;
                y = translation.get(2) * -1;
                DbgLog.msg("[Phoenix] y: " + y);

                double angle = Math.toDegrees(Math.atan2(x, y));
                DbgLog.msg("[Phoenix] angle: " + angle);
                opMode.telemetry.addData("angle ", angle);

                double frontLeftPower = power; //we need to determine if we need to adjust the left front power to adjust for direction to the right
                double backLeftPower = power; //this is to increase the back left if we need to go left
                double frontRightPower = power; //need to increase this power if need to move the left
                double backRightPower = power; //need to increase this power if need to move the right

                double powerAdjustment = 0;
                if (Math.abs(angle) > 1) //direction has change more than 1 degree, let's calculate how much power we need to adjust
                    powerAdjustment = (Math.abs(angle)) * adjustmentUnit;

                if (direction == TurnDirection.LEFT) {
                    if (angle > 1) {//direction has moved to the left more than 1 degree, let's adjust
                        frontLeftPower = frontLeftPower + powerAdjustment * 3;
                        frontRightPower = frontRightPower + powerAdjustment * 3;
                    }
                    else if (angle < -1) { //direction has move to the right more than 1 degree
                        backLeftPower = backLeftPower + powerAdjustment;
                        backRightPower = backRightPower + powerAdjustment;
                    }

                    backLeftPower = backLeftPower * -1;  //strafing left, the power of back left is oposite of front left
                    frontRightPower = frontRightPower * -1;  //strafing left, the power of the front Right is opposite of back right
                }
                else {
                    if (angle < -1) { //direction has moved to the left more than 2 degree, let's adjust
                        backRightPower = backRightPower + powerAdjustment;
                        backLeftPower = backLeftPower + powerAdjustment;
                    }
                    else if (angle > 1) {//direction has move to the right more than 2 degree
                        frontRightPower = frontRightPower + powerAdjustment;
                        frontLeftPower = frontLeftPower + powerAdjustment;
                    }

                    frontLeftPower = frontLeftPower * -1;
                    backRightPower = backRightPower * -1;
                }

                //if any of wheel is higher than 1.0, we need to reduce the power of all wheels proportionally.
                double mv = max(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backRightPower), Math.abs(backLeftPower));
                if (mv > 1) {
                    frontLeftPower = frontLeftPower * Math.abs(frontLeftPower / mv);
                    frontRightPower = frontRightPower * Math.abs(frontRightPower / mv);
                    backLeftPower = backLeftPower * Math.abs(backLeftPower / mv);
                    backRightPower = backRightPower * Math.abs(backRightPower / mv);
                }

                frontLeft.setPower(frontLeftPower);
                backLeft.setPower(backLeftPower);
                frontRight.setPower(frontRightPower);
                backRight.setPower(backRightPower);
            }

            opMode.idle();
        }


        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void strafe(int distance, int speed, TurnDirection direction, VuforiaTrackableDefaultListener image, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double adjustmentUnit = 50;
        DbgLog.msg("[Phoenix] adjustmentUnit: " + adjustmentUnit);

        OpenGLMatrix pos = image.getPose();
        VectorF translation;
        double x = 0;
        double y = distance;

        while(opMode.opModeIsActive() && y >= (double) distance) {
            pos = image.getPose();

            if(pos != null) {
                translation = pos.getTranslation();

                x = translation.get(0) * -1;
                y = translation.get(2) * -1;
                DbgLog.msg("[Phoenix] y: " + y);

                double angle = Math.toDegrees(Math.atan2(x, y));
                DbgLog.msg("[Phoenix] angle: " + angle);
                opMode.telemetry.addData("angle ", angle);

                int frontLeftSpeed = Math.abs(speed); //we need to determine if we need to adjust the left front power to adjust for direction to the right
                int backLeftSpeed = Math.abs(speed); //this is to increase the back left if we need to go left
                int frontRightSpeed = Math.abs(speed); //need to increase this power if need to move the left
                int backRightSpeed= Math.abs(speed);

                int powerAdjustment = 0;
                if (Math.abs(angle) > 1) //direction has change more than 1 degree, let's calculate how much power we need to adjust
                    powerAdjustment = (int)((Math.abs(angle)) * adjustmentUnit);

                if (direction == TurnDirection.LEFT) {
                    if (angle > 1) {//direction has moved to the left more than 1 degree, let's adjust
                        frontLeftSpeed = frontLeftSpeed + powerAdjustment;
                        frontRightSpeed = frontRightSpeed + powerAdjustment;
                    }
                    else if (angle < -1) { //direction has move to the right more than 1 degree
                        backLeftSpeed = backLeftSpeed + powerAdjustment;
                        backRightSpeed = backRightSpeed + powerAdjustment;
                    }
                }
                else {
                    if (angle < -1) { //direction has moved to the left more than 2 degree, let's adjust
                        backRightSpeed = backRightSpeed + powerAdjustment;
                        backLeftSpeed = backLeftSpeed + powerAdjustment;
                    }
                    else if (angle > 1) {//direction has move to the right more than 2 degree
                        frontRightSpeed = frontRightSpeed + powerAdjustment;
                        frontLeftSpeed = frontLeftSpeed + powerAdjustment;
                    }
                }

                //if any of wheel is higher than 1.0, we need to reduce the power of all wheels proportionally.
                int mv = max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed), Math.abs(backRightSpeed), Math.abs(backLeftSpeed));
                if (mv > 2400) {
                    frontLeftSpeed = frontLeftSpeed * 2400 / mv;
                    frontRightSpeed = frontRightSpeed * 2400 / mv;
                    backLeftSpeed = backLeftSpeed * 2400 / mv;
                    backRightSpeed = backRightSpeed * 2400 / mv;
                }

                frontLeft.setMaxSpeed(frontLeftSpeed);
                backLeft.setMaxSpeed(backLeftSpeed);
                frontRight.setMaxSpeed(frontRightSpeed);
                backRight.setMaxSpeed(backRightSpeed);

                if (direction == TurnDirection.LEFT) {
                    frontLeft.setPower(-1);
                    backLeft.setPower(1);
                    frontRight.setPower(1);
                    backRight.setPower(-1);
                } else {
                    frontLeft.setPower(1);
                    backLeft.setPower(-1);
                    frontRight.setPower(-1);
                    backRight.setPower(1);
                }
            }

            opMode.idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void driveUntilWhite(double power, OpticalDistanceSensor opt, LinearOpMode opMode) {
        opt.enableLed(true);

        double sensorValue = opt.getLightDetected();

        while(sensorValue <= 0 && opMode.opModeIsActive()) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);

            sensorValue = opt.getLightDetected();
            DbgLog.msg("[Phoenix] opt val: " + sensorValue);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
}

