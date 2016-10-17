package FlamingPhoenix;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoArm {
    private Servo baseServo; //the servo at the arm's base
    private Servo forwardServo; //the servo at the joint
    private Servo rotationServo;

    private double moveIncrement;
    private boolean isInPrepRed;
    private boolean isInPrepBlue;
    private boolean isInPrepArm;
    private int prepStep;

    public ServoArm(Servo base, Servo joint, Servo rot) {
        baseServo = base;
        forwardServo = joint;
        rotationServo = rot;
        baseServo.setPosition(1);
        forwardServo.setPosition(0);
        rotationServo.setPosition(.5);
        moveIncrement = 0.004;
        isInPrepRed = false;
        isInPrepBlue = false;
        isInPrepArm = false;
        prepStep = 0;
    }
    //This is intended to be used by TeleOps
    public void operateByRemote(Gamepad gamepad1, OpMode opModeInstance) {
        double basePosition = baseServo.getPosition();
        double forwardPosition = forwardServo.getPosition();
        double v = rotationServo.getPosition();

        if (gamepad1.dpad_down) {
            basePosition += moveIncrement;
            if (basePosition <= 1.0)
                baseServo.setPosition(basePosition);
        }
        else if (gamepad1.dpad_up) {
            basePosition -= moveIncrement;
            if (basePosition >= 0)
                baseServo.setPosition(basePosition);
        }

        if (gamepad1.y) {
            forwardPosition += moveIncrement;
            if (forwardPosition <= 1.0)
                forwardServo.setPosition(forwardPosition);
        } else if (gamepad1.a) {
            forwardPosition -= moveIncrement;
            if (forwardPosition >= 0)
                forwardServo.setPosition(forwardPosition);
        }

        if ((gamepad1.b) || (isInPrepRed)) {
           prepRed(); //Driver has pressed the red, this is to prepare the arm to right position without creating a lot of force to damage the servo
        }
        else if ((gamepad1.x) || (isInPrepBlue))
            prepBlue();
        else {
            if (gamepad1.dpad_left) {
                if(v <= 1)
                    v = rotationServo.getPosition() - 0.005;
            } else if (gamepad1.dpad_right) {
                if(v >= 0)
                    v = rotationServo.getPosition() + 0.005;
            }
            if(v >= 0 && v <= 1)
                rotationServo.setPosition(v);
        }
    }
    private void prepBlue() {
        double fortyFive = 0.6;
        if (!isInPrepBlue) {
            isInPrepBlue = true;
            prepStep = 0;
        }

        double rotationServoPosition = rotationServo.getPosition();
        DbgLog.msg("[Phoenix] prepBlue.rotationServoPosition:" + Double.toString(rotationServoPosition));
        if (rotationServoPosition > fortyFive) {  //the following is to move the arm 45 degree to the right
            rotationServoPosition -= moveIncrement;
            if (rotationServoPosition < fortyFive)
                rotationServoPosition = fortyFive;
            rotationServo.setPosition(rotationServoPosition);
        }
        else if (rotationServoPosition < 0.598) {
            rotationServoPosition += moveIncrement;
            if (rotationServoPosition > fortyFive)
                rotationServoPosition = fortyFive;

            rotationServo.setPosition(rotationServoPosition);
        }
        else if (prepStep <= 3) {
            prepArm();
        }
        else {
            isInPrepBlue = false;
            isInPrepArm = false;
        }
    }
    //Prepare the arm for the red side of all clear.  This is intended for Teleops
    private void prepRed() {
        double fortyFive = 0.3;
        if (!isInPrepRed) {
            isInPrepRed = true;
            prepStep = 0;
        }

        double rotationServoPosition = rotationServo.getPosition();
        DbgLog.msg("[Phoenix] prepRed.rotationServoPosition:" + Double.toString(rotationServoPosition));
        if (rotationServoPosition > fortyFive) {  //the following is to move the arm 45 degree to the left
            rotationServoPosition -= moveIncrement;
            if (rotationServoPosition < fortyFive)
                rotationServoPosition = fortyFive;
            rotationServo.setPosition(rotationServoPosition);
        }
        else if (rotationServoPosition < 0.298) {
            rotationServoPosition += moveIncrement;
            if (rotationServoPosition > fortyFive)
                rotationServoPosition = fortyFive;

            rotationServo.setPosition(rotationServoPosition);
        }
        else if (prepStep <= 3) {
           prepArm();
        }
        else {
            isInPrepRed = false;
            isInPrepArm = false;
        }
    }
    private void prepArm() {
        if (!isInPrepArm) {
            isInPrepArm = true;
            prepStep = 1;
        }

        if (prepStep == 1)
            prepArmStep1();
        else if (prepStep == 2)
            prepArmStep2();
        else if (prepStep == 3)
            prepArmStep3();
        else if (prepStep > 3)
            isInPrepArm = false;
    }

    private void prepArmStep1() {
        double baseServoPosition = baseServo.getPosition();

        if (baseServoPosition > 0.3) {
            baseServoPosition -= moveIncrement;
            if (baseServoPosition < 0.3) {
                baseServoPosition = 0.3;
            }

            baseServo.setPosition(baseServoPosition);
        }
        else
            prepStep = 2;
    }

    private void prepArmStep2() {
        double forwardServoPosition = forwardServo.getPosition();
        DbgLog.msg("[Phoenix] prepArmStep2.currentForwardServoPosition:" + Double.toString(forwardServoPosition));

        if (forwardServoPosition < 0.698) {
            forwardServoPosition += moveIncrement;
            if (forwardServoPosition > 0.7) {
                forwardServoPosition = 0.7;
            }

            forwardServo.setPosition(forwardServoPosition);
            DbgLog.msg("[Phoenix] prepArmStep2. forwardServoPosition:" + Double.toString(forwardServoPosition));
        }
        else {
            prepStep = 3;
            DbgLog.msg("[Phoenix] Done prepArmStep2");
        }
    }

    private void prepArmStep3() {
        double forwardServoPosition = forwardServo.getPosition();
        double baseServoPosition = baseServo.getPosition();

        //Wait until the forward arm is high up before moving the base to prevent moving all-clear to opponent
        if (forwardServoPosition > 0.6) {
            if (baseServoPosition > 0) {
                baseServoPosition -= moveIncrement;
                if (baseServoPosition < 0)
                    baseServoPosition = 0;
            }

            baseServo.setPosition(baseServoPosition);
        }

        if (forwardServoPosition < 0.8) {
            forwardServoPosition += moveIncrement;
            if (forwardServoPosition > 0.8)
                forwardServoPosition = 0.8;

            forwardServo.setPosition(forwardServoPosition);
        }

        if ((baseServoPosition == 0) && (forwardServoPosition >= 0.8))
            prepStep = 4;
    }

    //if position is not >= 0.8, it is not safe because it may actually tip all-clear to the opponent
    private boolean isTeleOpsSafe(double forwardPosition) {
        return (forwardPosition >= 0.8);
    }

    public void moveForwardArm(double position, boolean fast, LinearOpMode opModeInstance) throws InterruptedException {
        double v;
        double moveIncrement = 0.004;

        if(fast)
            moveIncrement = 0.006;

        if (forwardServo.getPosition() < position) {
            while (forwardServo.getPosition() < position) {
                v = forwardServo.getPosition() + moveIncrement;
                forwardServo.setPosition(v);
                opModeInstance.waitForNextHardwareCycle();
            }
        }
        else {
            while (forwardServo.getPosition() > position) {
                v = forwardServo.getPosition() - moveIncrement;
                forwardServo.setPosition(v);
                opModeInstance.waitForNextHardwareCycle();
            }
        }
    }

    public void moveBaseArm(double position, boolean moveOtherArm, LinearOpMode opModeInstance) throws InterruptedException {
        double v;

        double basePosition = baseServo.getPosition();
        double keepPosition = forwardServo.getPosition();
        double forwardPosition = keepPosition;

        if (basePosition > position) {
            while (basePosition > position) {
                basePosition = basePosition - moveIncrement;
                baseServo.setPosition(basePosition);

                if(moveOtherArm){
                    if ((1- basePosition + keepPosition) <= 1)
                        forwardPosition = 1- basePosition + keepPosition;
                    opModeInstance.telemetry.addData("forward:", forwardPosition);
                    opModeInstance.telemetry.addData("base:", basePosition);
                    if (forwardPosition <= 1)
                        forwardServo.setPosition(forwardPosition);
                }
                opModeInstance.waitForNextHardwareCycle();
            }
        }
        else {
            while (baseServo.getPosition() < position) {
                v = baseServo.getPosition() + moveIncrement;
                baseServo.setPosition(v);
                if(moveOtherArm){
                    double x = forwardServo.getPosition() - (1 - v);
                    if(x > 0)
                        forwardServo.setPosition(x);
                }
                opModeInstance.waitForNextHardwareCycle();
            }
        }
    }

    public void setDirection(double position, LinearOpMode opModeInstance) throws InterruptedException {
        double moveIncrement = 0.005;
        double v = 0;

        if (rotationServo.getPosition() < position) {
            while (rotationServo.getPosition() < position) {
                v = rotationServo.getPosition() + moveIncrement;
                if(v >= 0 && v <= 1)
                    rotationServo.setPosition(v);
                else {
                    v = rotationServo.getPosition() - moveIncrement;
                    rotationServo.setPosition(v);
                }
                opModeInstance.waitForNextHardwareCycle();
            }
        }
        else {
            while (rotationServo.getPosition() > position) {
                v = rotationServo.getPosition() - moveIncrement;

                if(v >= 0 && v <= 1)
                    rotationServo.setPosition(v);
                else {
                    v = (int)Math.round(v);
                    rotationServo.setPosition(v);
                }

                opModeInstance.waitForNextHardwareCycle();
            }
        }
    }

    //perform sequence of steps to put climbers in the bucket
    public void putClimberInBucket(LinearOpMode opModeInstance) throws InterruptedException{
        this.setDirection(0, opModeInstance); //rotate arms to the side
        Thread.sleep(700);
        this.moveBaseArm(0.45, true, opModeInstance); //raise the base arm
        Thread.sleep(100);
        this.moveForwardArm(.40, true, opModeInstance); //lower the forearm to allow the climber to drop
        Thread.sleep(500);
        this.moveBaseArm(.98, false, opModeInstance);  //lower the arm back
        this.moveForwardArm(.1, false, opModeInstance); //move back the forearm
        this.setDirection(0.5, opModeInstance); //rotate arm back to the middle
    }

    public void putClimberInBucketFromLeft(LinearOpMode opModeInstance) throws InterruptedException{
        this.setDirection(.75, opModeInstance); //rotate arms to the side
        Thread.sleep(700);
        this.moveBaseArm(0.30, true, opModeInstance); //raise the base arm
        Thread.sleep(100);
        this.moveForwardArm(.45, true, opModeInstance); //lower the forearm to allow the climber to drop
        Thread.sleep(500);
        this.moveBaseArm(.98, false, opModeInstance);  //lower the arm back
        this.moveForwardArm(.1, false, opModeInstance); //move back the forearm
        this.setDirection(0.5, opModeInstance); //rotate arm back to the middle
    }
    public void putClimberInBucketFromRight(LinearOpMode opModeInstance) throws InterruptedException{
        this.setDirection(.25, opModeInstance); //rotate arms to the side
        Thread.sleep(700);
        this.moveBaseArm(0.30, true, opModeInstance); //raise the base arm
        Thread.sleep(100);
        this.moveForwardArm(.45, true, opModeInstance); //lower the forearm to allow the climber to drop
        Thread.sleep(500);
        this.moveBaseArm(.98, false, opModeInstance);  //lower the arm back
        this.moveForwardArm(.1, false, opModeInstance); //move back the forearm
        this.setDirection(0.5, opModeInstance); //rotate arm back to the middle
    }

}