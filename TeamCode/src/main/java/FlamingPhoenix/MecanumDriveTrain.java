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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
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

    public MecanumDriveTrain(String FrontLeftName, String FrontRightName, String BackLeftName, String BackRightName, ModernRoboticsI2cGyro gyroscope, OpMode OperatorMode) throws InterruptedException {
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

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(40);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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


    public void Drive(Gamepad gamePad, float speedMultiplier) {
        speedMultiplier = Math.abs(speedMultiplier);
        if (speedMultiplier > 1)
            speedMultiplier = 1;
        else if (Math.abs(gamePad.left_stick_x) > .8)
            speedMultiplier= 1;


        float x1;
        float y1;
        float x2;

        if ((gamePad.right_trigger > 0.5) && (gamePad.left_trigger > 0.5))  //both trigger are pressed, let's slow down a lot more
        {
            x1 = gamePad.left_stick_x * 0.25f;
            y1 = gamePad.left_stick_y * 0.25f;
            x2 = gamePad.right_stick_x * 0.25f;
        } else if ((gamePad.right_trigger > 0.5) || (gamePad.left_trigger > 0.5)) { //only one if the trigger is pressed, let's slow down by 50%
            x1 = gamePad.left_stick_x * 0.5f;
            y1 = gamePad.left_stick_y * 0.5f;
            x2 = gamePad.right_stick_x * 0.5f;
        }
        else {
            x1 = gamePad.left_stick_x * speedMultiplier;
            y1 = gamePad.left_stick_y * speedMultiplier;
            x2 = gamePad.right_stick_x * speedMultiplier;
        }

        mecanumDrive(x1, y1, x2);
    }

    /*
     * Control or drive the robot using gamepad
     * @param gamePad
     */
    public void Drive(Gamepad gamePad) {
        Drive(gamePad, 1f);
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

    /*public void drive(int d, double power, LinearOpMode opMode) throws InterruptedException {
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
    }*/

    public void resetMotorSpeed() {
        backLeft.setMaxSpeed(2400);
        backRight.setMaxSpeed(2400);
        frontLeft.setMaxSpeed(2400);
        frontRight.setMaxSpeed(2400);
    }

    //Drive by using PIDControl - Run to Position
    public void drive(double d, Direction direction, double power, int timeout, LinearOpMode opMode) throws InterruptedException {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int pulseNeeded = (int) Math.round(((double) encoderPPR * d) / (wheelDiameter * Math.PI));
        if (direction == Direction.BACKWARD) {
            pulseNeeded = pulseNeeded * -1;
            power = Math.abs(power) * -1;
        }
        else
            power = Math.abs(power);

        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);

        runtime.reset();
        int currentEncoderTick = backRight.getCurrentPosition();
        DbgLog.msg("[Phoenix.Drive] pulseNeeded: %d, starting tick = %d ",pulseNeeded, currentEncoderTick);
        while ((opMode.opModeIsActive()) && (runtime.seconds() <= timeout) && (Math.abs(currentEncoderTick) < Math.abs(pulseNeeded))) {
            currentEncoderTick = backRight.getCurrentPosition();
            if ((Math.abs(pulseNeeded) > 200) && (Math.abs(pulseNeeded) - Math.abs(currentEncoderTick)) <= 200) {
                if (direction == Direction.FORWARD) {
                    backLeft.setPower(0.1);
                    backRight.setPower(0.1);
                    frontLeft.setPower(0.1);
                    frontRight.setPower(0.1);
                } else {
                    backLeft.setPower(-0.1);
                    backRight.setPower(-0.1);
                    frontLeft.setPower(-0.1);
                    frontRight.setPower(-0.1);
                }
            }
        }

        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

        DbgLog.msg("[Phoenix.Drive] backLeft" + backLeft.getCurrentPosition());
        DbgLog.msg("[Phoenix.Drive] backRight" + backRight.getCurrentPosition());
        DbgLog.msg("[Phoenix.Drive] frontLeft" + frontLeft.getCurrentPosition());
        DbgLog.msg("[Phoenix.Drive] frontRight" + frontRight.getCurrentPosition());

    }

    //Drive by controlling the motor rotation speed using encoder
    @Deprecated()
    public void drive(double d, Direction direction, int speed, LinearOpMode opMode) throws InterruptedException {

        speed = Math.abs(speed); //let's make sure it is positive value

        int pulseNeeded = (int) (Math.round(((double) encoderPPR * d) / (wheelDiameter * Math.PI)));
        double power = 1;
        if (direction == Direction.BACKWARD)
            power = -1;

        int currentTicks = backLeft.getCurrentPosition();
        while ((Math.abs(currentTicks) < pulseNeeded) && opMode.opModeIsActive()) {
            try {
                currentTicks = backLeft.getCurrentPosition();
                opMode.telemetry.addData("Encoder: ", currentTicks);

                if ((Math.abs(currentTicks) <= 400) && (speed > 600)) { //start slow
                    backLeft.setMaxSpeed(600);
                    backRight.setMaxSpeed(600);
                    frontLeft.setMaxSpeed(600);
                    frontRight.setMaxSpeed(600);
                } else if (((pulseNeeded - Math.abs(currentTicks)) < 400) && (speed > 800)) { //slow down
                    backLeft.setMaxSpeed(600);
                    backRight.setMaxSpeed(600);
                    frontLeft.setMaxSpeed(600);
                    frontRight.setMaxSpeed(600);
                    DbgLog.msg("[Phoenix:Drive] Almost reached; with original speed " + speed);

                } else {
                    backLeft.setMaxSpeed(speed);
                    backRight.setMaxSpeed(speed);
                    frontLeft.setMaxSpeed(speed);
                    frontRight.setMaxSpeed(speed);
                }

                frontLeft.setPower(power);
                frontRight.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
            }
            catch (NullPointerException e) {
                DbgLog.msg("[Phoenix:Drive] you got a null pointer exception");
            }
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        DbgLog.msg("[Phoenix:Drive] Reached destination");
    }

    public void turnWithGyro(int degree, double power, TurnDirection direction, ModernRoboticsI2cGyro gyro, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMaxSpeed(2400);
        backRight.setMaxSpeed(2400);
        frontLeft.setMaxSpeed(2400);
        frontRight.setMaxSpeed(2400);

        int startHeading = gyro.getIntegratedZValue();
        int targetHeading = startHeading + (direction == TurnDirection.RIGHT ? degree * -1 : degree);

        DbgLog.msg("[Phoenix:Turn] targetHeading = " + targetHeading);

        double speed = Math.abs(power);

        int currentHeading = gyro.getIntegratedZValue();
        DbgLog.msg("[Phoenix] currentHeading = " + currentHeading);

        if(direction == TurnDirection.RIGHT) { //negative means turning right and vice versa
            while((currentHeading > targetHeading) && (opMode.opModeIsActive())) {
                if((Math.abs(currentHeading - targetHeading) < 10) && degree > 10)
                    break;

                if (Math.abs(currentHeading - targetHeading) < 20) {
                    speed = 0.20;
                    //DbgLog.msg("[Phoenix:Turn] cut the speed to = " + speed);
                }

                frontRight.setPower(speed * -1);
                backRight.setPower(speed * -1);
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                currentHeading = gyro.getIntegratedZValue();

                //DbgLog.msg("[Phoenix:Turn] currentHeading = " + currentHeading);
            }
        } else {
            DbgLog.msg("[Phoenix:Turn] Turning Left, speed = %6.3f ", speed);
            while((currentHeading < targetHeading) && (opMode.opModeIsActive())){
                if((Math.abs(targetHeading - currentHeading) < 10) && degree > 10)
                    break;

                if (Math.abs(currentHeading - targetHeading) < 20) {
                    speed = 0.20;
                    //DbgLog.msg("[Phoenix:Turn] cut the speed to = " + speed);
                }

                frontRight.setPower(speed);
                backRight.setPower(speed);
                frontLeft.setPower(speed * -1);
                backLeft.setPower(speed * -1);
                currentHeading = gyro.getIntegratedZValue();
            }
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        DbgLog.msg("[Phoenix:Turn] final1currentHeading = " + Double.toString(gyro.getIntegratedZValue()));
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
    private int max(int... args) {
        int m = 0;

        for (int i = 0; i < args.length; i++) {
            if (args[i] > m)
                m = args[i];
        }

        return m;
    }

    public void strafe(int distance, double power, TurnDirection direction, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

        int pulseNeeded = (int) Math.round((encoderPPR * distance) / (wheelDiameter * Math.PI));

        pulseNeeded = (int) Math.round((double) pulseNeeded/0.5); //the distance is around .5 the normal

        while(((Math.abs(backLeft.getCurrentPosition())) < pulseNeeded) && opMode.opModeIsActive()) {
            if (direction == TurnDirection.LEFT) {
                frontLeft.setPower(-power);
                backLeft.setPower(power);
                frontRight.setPower(power);
                backRight.setPower(-power);
            } else {
                frontLeft.setPower(power);
                backLeft.setPower(-power);
                frontRight.setPower(-power);
                backRight.setPower(power);
            }
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void strafe(int distance, double power, int timeout, TurnDirection direction, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

        int pulseNeeded = (int) Math.round((encoderPPR * distance) / (wheelDiameter * Math.PI));

        pulseNeeded = (int) Math.round((double) pulseNeeded/0.5); //the distance is around .5 the normal

        runtime.reset();
        while(((Math.abs(backLeft.getCurrentPosition())) < pulseNeeded) && opMode.opModeIsActive() && runtime.seconds() < timeout) {
            if (direction == TurnDirection.LEFT) {
                frontLeft.setPower(-power);
                backLeft.setPower(power);
                frontRight.setPower(power);
                backRight.setPower(-power);
            } else {
                frontLeft.setPower(power);
                backLeft.setPower(-power);
                frontRight.setPower(-power);
                backRight.setPower(power);
            }
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public boolean strafe(int distance, int speed, TurnDirection direction, VuforiaTrackable imageObject, LinearOpMode opMode) throws InterruptedException {

        VuforiaTrackableDefaultListener image = (VuforiaTrackableDefaultListener) imageObject.getListener();

        double adjustmentUnit = 50;
        DbgLog.msg("[Phoenix] adjustmentUnit: " + adjustmentUnit);

        VectorF translation;
        double x = 0;
        double y = distance;

        int i = 0;

        if(image.getPose() == null) {
            opMode.sleep(500);

            DbgLog.msg("[Phoenix image] first try, cannot see " + imageObject.getName());

            if (image.getPose() == null) {
                DbgLog.msg("[Phoenix image] cannot see " + imageObject.getName());

                return false; //Camera does not see image, return false
            }
        }

        while(opMode.opModeIsActive() && y >= (double) distance && i <= 50) {
            OpenGLMatrix pos = image.getPose();

            if(pos != null) {
                DbgLog.msg("[Phoenix image] sees " + imageObject.getName());
                i = 0;
                translation = pos.getTranslation();

                x = translation.get(0) * -1;
                y = translation.get(2) * -1;
                DbgLog.msg("[Phoenix] y: " + y);

                double angle = Math.toDegrees(Math.atan2(x, y));
                DbgLog.msg("[Phoenix image] angle: " + angle);
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
                        backLeftSpeed += powerAdjustment;
                        backRightSpeed += powerAdjustment;
                    }
                    else if (angle < -1) { //direction has move to the right more than 1 degree
                        frontLeftSpeed += powerAdjustment;
                        frontRightSpeed += powerAdjustment;
                    }
                }
                else {
                    if (angle < -1) { //direction has moved to the left more than 2 degree, let's adjust
                        frontRightSpeed += powerAdjustment;
                        frontLeftSpeed += powerAdjustment;
                    }
                    else if (angle > 1) {//direction has move to the right more than 2 degree
                        backRightSpeed += powerAdjustment;
                        backLeftSpeed += powerAdjustment;
                    }
                }

                //if any of wheel is higher than 1.0, we need to reduce the power of all wheels proportionally.
                int mv = max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed), Math.abs(backRightSpeed), Math.abs(backLeftSpeed));
                if (mv > 2400) {
                    frontLeftSpeed = (int) (Math.round((double) frontLeftSpeed / (double) mv));
                    frontRightSpeed = (int) Math.round((double) frontRightSpeed  / (double) mv);
                    backLeftSpeed = (int) Math.round((double) backLeftSpeed / (double) mv);
                    backRightSpeed = (int) Math.round((double) backRightSpeed / (double) mv);

                    DbgLog.msg("[Phoenix] adjusted frontLeft: " + frontLeftSpeed);
                    DbgLog.msg("[Phoenix] adjusted frontRight: " + frontRightSpeed);
                    DbgLog.msg("[Phoenix] adjusted backLeft: " + backLeftSpeed);
                    DbgLog.msg("[Phoenix] adjusted backRight: " + backRightSpeed);
                }

                try {
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

                } catch (Exception e) {
                    DbgLog.msg("[Phoenix image] null point exception here while tracking " + imageObject.getName());
                }
            }
            else {
                i++;

                DbgLog.msg("[Phoenix image] stop seeing " + imageObject.getName());
                DbgLog.msg("[Phoenix image] last y= " + Double.toString(y));
            }
            DbgLog.msg("[Phoenix image] i = " + i);
            opMode.idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        DbgLog.msg("[Phoenix image] i done = " + i);
        return true;
    }

    //Use power to control the robot by following Vuforia object
    public double strafe(int distance, double power, TurnDirection direction, VuforiaTrackable imageObject, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

        VuforiaTrackableDefaultListener image = (VuforiaTrackableDefaultListener) imageObject.getListener();

        double xAdjustmentUnit = 0.0005;
        double angleAdjustmentUnit = 0.06;
        double angleAdjustment = 0;
        double angleDifference = 0;


        DbgLog.msg("[Phoenix:strafe] adjustmentUnit: " + xAdjustmentUnit);

        VectorF translation;
        double x = 0;
        double y = distance;

        int i = 0;

        if(image.getPose() == null) {
            opMode.sleep(500);

            DbgLog.msg("[Phoenix:strafe] image first try, cannot see " + imageObject.getName());

            if (image.getPose() == null) {
                DbgLog.msg("[Phoenix:strafe] image cannot see " + imageObject.getName());

                return -9999; //Camera does not see image, return false
            }
        }

        while(opMode.opModeIsActive() && y >= (double) distance && i <= 50) {
            OpenGLMatrix pos = image.getPose();

            if(pos != null) {
                DbgLog.msg("[Phoenix:strafe] sees image " + imageObject.getName());
                i = 0;
                translation = pos.getTranslation();

                x = translation.get(0) * -1;
                y = translation.get(2) * -1;

                double angle = Math.toDegrees(Math.atan2(x, y));
                DbgLog.msg("[Phoenix:strafe] image angle: " + angle);
                opMode.telemetry.addData("angle ", angle);

                double frontLeftSpeed = Math.abs(power); //we need to determine if we need to adjust the left front power to adjust for direction to the right
                double backLeftSpeed = Math.abs(power); //this is to increase the back left if we need to go left
                double frontRightSpeed = Math.abs(power); //need to increase this power if need to move the left
                double backRightSpeed= Math.abs(power);

                double xPowerAdjustment = 0;

                if (x > 15) //distance is more than 15 mm, lets adjust the power proportionally, Robot is Right side of the image
                    xPowerAdjustment = xAdjustmentUnit * Math.abs(x);
                else if (x < -15)
                    xPowerAdjustment = -30 * xAdjustmentUnit * Math.abs(x);
                else
                    xPowerAdjustment = 0;

                DbgLog.msg("[Phoenix:Strafe] x=%7.2f powerAdjustment=%5.2f", x, xPowerAdjustment);

                //Adjust the power to turn the robot so it would be perpendecular to the image
                double zAngle = (double) MyUtility.getImageAngle(imageObject);
                if (Math.abs(90 - zAngle) > 1)  //The robot is no longer parallel to beacon
                {
                    if ((angleDifference > 0 && (90-zAngle) <0) || (angleDifference < 0 && (90-zAngle) >0))
                        angleAdjustmentUnit = angleAdjustmentUnit / 2.0;
                    else if (Math.abs(90 - zAngle) > (Math.abs(angleDifference) + 4))
                        angleAdjustmentUnit = angleAdjustmentUnit * 2.0;

                    angleDifference = 90 - zAngle;
                    angleAdjustment = Math.abs(angleDifference) * angleAdjustmentUnit;
                }
                else
                    angleAdjustment = 0;

                if(angleAdjustment > .35)
                    angleAdjustment = .35;

                DbgLog.msg("[Phoenix:Strafe] zAngle=%5.2f  angleAdjustment=%5.2f angleDifference=%5.2f angleAdjustmentUnit=%7.5f", zAngle, angleAdjustment, angleDifference, angleAdjustmentUnit);

                try {
                    if (direction == TurnDirection.LEFT) {

                        /*
                        if (zAngle < 85)  //need to turn right by giving more power to the back wheels
                        {
                            backLeftSpeed += angleAdjustment;
                            backRightSpeed += angleAdjustment;
                        }
                        else if (zAngle > 95)  //need to turn left by giving more power to the front wheels
                        {
                            frontLeftSpeed += angleAdjustment;
                            frontRightSpeed += angleAdjustment;
                        }
                        else {
                            frontLeftSpeed = frontLeftSpeed + (-1 * xPowerAdjustment);
                            frontRightSpeed += xPowerAdjustment;
                            backLeftSpeed += xPowerAdjustment;
                            backRightSpeed = backRightSpeed + (-1 * xPowerAdjustment);
                        }

                        if (max(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed) > 1) {
                            double mv = max(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
                            frontLeftSpeed = frontLeftSpeed * (frontLeftSpeed /mv);
                            frontRightSpeed = frontRightSpeed * (frontRightSpeed /mv);
                            backLeftSpeed =backLeftSpeed * (backLeftSpeed/mv);
                            backRightSpeed =backRightSpeed * (backRightSpeed/mv);
                        }
                        */

                        DbgLog.msg("[Phoenix:Strafe] frontLeft=%7.4f backLeft=%7.4f frontRight=%7.4f backRight=%7.4f", frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);

                        frontLeft.setPower(-1 * frontLeftSpeed);
                        backLeft.setPower(1 * backLeftSpeed);
                        frontRight.setPower(1 * frontRightSpeed);
                        backRight.setPower(-1 * backRightSpeed);
                    } else {
                        frontLeftSpeed += xPowerAdjustment;
                        frontRightSpeed = frontLeftSpeed * -1 + xPowerAdjustment;
                        backLeftSpeed = backLeftSpeed *-1 + xPowerAdjustment;
                        backRightSpeed += xPowerAdjustment;

                        frontLeft.setPower(1 * frontLeftSpeed);
                        backLeft.setPower(-1 * backLeftSpeed);
                        frontRight.setPower(-1 * frontRightSpeed);
                        backRight.setPower(1 * backRightSpeed);
                    }

                } catch (Exception e) {
                    DbgLog.msg("[Phoenix:strafe] image null point exception here while tracking %s, message=%s", imageObject.getName(), e.getMessage());
                }
            }
            else {
                i++;

                DbgLog.msg("[Phoenix:strafe] image stop seeing " + imageObject.getName());
                DbgLog.msg("[Phoenix:strafe] image last y= " + Double.toString(y));
            }
            opMode.idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        DbgLog.msg("[Phoenix:strafe] i done = " + i);
        return (x * -1); //return last x position;
    }

    public boolean strafe(int distance, int speed, TurnDirection direction, VuforiaTrackable imageObject, ModernRoboticsI2cGyro gyro, LinearOpMode opMode) throws InterruptedException {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.idle();

        VuforiaTrackableDefaultListener image = (VuforiaTrackableDefaultListener) imageObject.getListener();

        int adjustmentUnit = 250;
        DbgLog.msg("[Phoenix:strafe] adjustmentUnit: " + adjustmentUnit);

        VectorF translation;
        double x = 0;
        double y = distance;

        int i = 0;

        if(image.getPose() == null) {
            opMode.sleep(500);

            DbgLog.msg("[Phoenix:strafe] image first try, cannot see " + imageObject.getName());

            if (image.getPose() == null) {
                DbgLog.msg("[Phoenix:strafe] image cannot see " + imageObject.getName());

                return false; //Camera does not see image, return false
            }
        }

        while(opMode.opModeIsActive() && y >= (double) distance && i <= 50) {
            OpenGLMatrix pos = image.getPose();

            if(pos != null) {
                DbgLog.msg("[Phoenix:strafe] sees image " + imageObject.getName());
                i = 0;
                translation = pos.getTranslation();

                x = translation.get(0) * -1;
                y = translation.get(2) * -1;

                double angle = Math.toDegrees(Math.atan2(x, y));
                DbgLog.msg("[Phoenix:strafe] image angle: " + angle);
                opMode.telemetry.addData("angle ", angle);

                int frontLeftSpeed = Math.abs(speed); //we need to determine if we need to adjust the left front power to adjust for direction to the right
                int backLeftSpeed = Math.abs(speed); //this is to increase the back left if we need to go left
                int frontRightSpeed = Math.abs(speed); //need to increase this power if need to move the left
                int backRightSpeed= Math.abs(speed);

                int powerAdjustment = 0;

                if (x > 15) //direction has change more than 1 degree, let's calculate how much power we need to adjust
                    powerAdjustment = adjustmentUnit;
                else if (x < -15)
                    powerAdjustment = -1 * adjustmentUnit;
                else
                    powerAdjustment = 0;

                DbgLog.msg("[Phoenix:Strafe] x=%7.2f powerAdjustment=%5d", x, powerAdjustment);

                try {
                    if (direction == TurnDirection.LEFT) {
                        frontLeftSpeed = frontLeftSpeed * -1 + powerAdjustment;
                        frontRightSpeed += powerAdjustment;
                        backLeftSpeed += powerAdjustment;
                        backRightSpeed = backRightSpeed * -1 + powerAdjustment;

                        frontLeft.setMaxSpeed(Math.abs(frontLeftSpeed));
                        backLeft.setMaxSpeed(Math.abs(backLeftSpeed));
                        frontRight.setMaxSpeed(Math.abs(frontRightSpeed));
                        backRight.setMaxSpeed(Math.abs(backRightSpeed));

                        frontLeft.setPower(-1);
                        backLeft.setPower(1);
                        frontRight.setPower(1);
                        backRight.setPower(-1);
                    } else {
                        frontLeftSpeed += powerAdjustment;
                        frontRightSpeed = frontLeftSpeed * -1 + powerAdjustment;
                        backLeftSpeed = backLeftSpeed *-1 + powerAdjustment;
                        backRightSpeed += powerAdjustment;

                        frontLeft.setMaxSpeed(Math.abs(frontLeftSpeed));
                        backLeft.setMaxSpeed(Math.abs(backLeftSpeed));
                        frontRight.setMaxSpeed(Math.abs(frontRightSpeed));
                        backRight.setMaxSpeed(Math.abs(backRightSpeed));

                        frontLeft.setPower(1);
                        backLeft.setPower(-1);
                        frontRight.setPower(-1);
                        backRight.setPower(1);
                    }

                } catch (Exception e) {
                    DbgLog.msg("[Phoenix:strafe] image null point exception here while tracking " + imageObject.getName());
                }
            }
            else {
                i++;

                DbgLog.msg("[Phoenix:strafe] image stop seeing " + imageObject.getName());
                DbgLog.msg("[Phoenix:strafe] image last y= " + Double.toString(y));
            }
            DbgLog.msg("[Phoenix:strafe] i = " + i);
            opMode.idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        DbgLog.msg("[Phoenix:strafe] i done = " + i);
        return true;
    }

    public void driveUntilWhite(double power, OpticalDistanceSensor opt, LinearOpMode opMode) {
        opt.enableLed(true);

        double sensorValue = opt.getLightDetected();
        DbgLog.msg("[Phoenix] opt val: " + sensorValue);

        while(sensorValue <= .1 && opMode.opModeIsActive()) {
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

    public void driveUntilImage(int distance, double power, Direction d, VuforiaTrackable imageObject, LinearOpMode opMode) {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

        DbgLog.msg("[Phoenix:driveuntilimage] Starting");

        int pulseNeeded = (int) Math.round((encoderPPR * distance) / (wheelDiameter * Math.PI));

        VuforiaTrackableDefaultListener image = (VuforiaTrackableDefaultListener) imageObject.getListener();

        power = (d == Direction.BACKWARD ? power * -1 : power * 1);

        DbgLog.msg("[Phoenix:driveuntilimage] pulseNeeded: " + pulseNeeded + ". backRight encoder value: " + backRight.getCurrentPosition());

        OpenGLMatrix pos = image.getPose();
        while (pos == null  && Math.abs(backRight.getCurrentPosition()) < pulseNeeded && opMode.opModeIsActive()) {
            backLeft.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(power);
            frontRight.setPower(power);
            opMode.idle();
            pos =image.getPose();
        }

        if (pos != null) {
            VectorF v = pos.getTranslation();
            float imageXPosition = v.get(0);

            DbgLog.msg("[Phoenix:driveuntilimage] imageXPosition= %7.2f ", imageXPosition);
            if(d == Direction.FORWARD) {
                if (v.get(0) < 0) {
                    while (v.get(0) < 0  && Math.abs(backRight.getCurrentPosition()) < pulseNeeded) {
                        backLeft.setPower(power);
                        backRight.setPower(power);
                        frontLeft.setPower(power);
                        frontRight.setPower(power);

                        pos = image.getPose();
                        if (pos != null)
                            v = pos.getTranslation();
                        else
                            v = new VectorF(0, 0, 0, 0);
                    }
                }
            }
            else {
                if (v.get(0) > 0) {
                    while (v.get(0) > 0 && Math.abs(backRight.getCurrentPosition()) < pulseNeeded) {
                        backLeft.setPower(power);
                        backRight.setPower(power);
                        frontLeft.setPower(power);
                        frontRight.setPower(power);

                        pos = image.getPose();
                        if (pos != null)
                            v = pos.getTranslation();
                        else
                            v = new VectorF(0, 0, 0, 0);
                    }
                }
            }
        }

        DbgLog.msg("[Phoenix:driveuntilimage] backRight final encoder value: " + backRight.getCurrentPosition());

        DbgLog.msg("I see the image");
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

    }

    public void turnAjdustment(double power, TurnDirection d, LinearOpMode opMode) {
        DbgLog.msg("[Phoenix:turnAdjustment] we made it");

        if(d == TurnDirection.RIGHT) { //negative means turning right and vice versa
                frontRight.setPower(power * -1);
                backRight.setPower(power * -1);
                frontLeft.setPower(power);
                backLeft.setPower(power);
        } else {
                frontRight.setPower(power);
                backRight.setPower(power);
                frontLeft.setPower(power * -1);
                backLeft.setPower(power * -1);
        }

        for(int i = 0; i <= 5; i++) {
            opMode.idle();
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }

}

