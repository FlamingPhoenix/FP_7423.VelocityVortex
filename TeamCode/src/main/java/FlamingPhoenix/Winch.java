package FlamingPhoenix;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Austin_2 on 11/26/2015.
 */
public class Winch {

    private Servo anchorServo;
    private Servo jointServo;
    private DcMotor winchMotor;
    private ServoController servCont;
    private boolean servoIsOn;
    private double currentJointPosition;

    private static final double l = 18.0;

    public Winch(Servo aServo, Servo jServo, DcMotor wMotor, ServoController sCont) {
        anchorServo = aServo;
        jointServo = jServo;
        winchMotor = wMotor;
        servCont = sCont;

        setJointPosition(.1);


    }

    private void turnServoOn(boolean onOrOff) {
        if (servCont != null) {
            if (onOrOff) {
                servCont.pwmEnable();
                servoIsOn = true;
            } else {
                servCont.pwmDisable();
                servoIsOn = false;
            }
        }
    }

    private void setJointPosition(double position) {
        jointServo.setPosition(position);
        currentJointPosition = position;
    }

    private void moveJoint(double yValue, OpMode OpModeInstance) {
        if (yValue < -0.5) {
            if(currentJointPosition > 0){
                //double p = Range.clip(currentJointPosition + 0.007, 0, .9);
                double p = currentJointPosition + 0.007;
                if(p < 1) {
                    setJointPosition(p);
                }
            }
        }
        else if(yValue > 0.5) {
            if(currentJointPosition < 1){
                //double p = Range.clip(currentJointPosition - 0.007, 0, .9);
                double p = currentJointPosition - 0.007;
                if(p > 0) {
                    setJointPosition(p);
                }
            }
        }

        OpModeInstance.telemetry.addData("yValue ", yValue);
        OpModeInstance.telemetry.addData("currentJointPosition:", currentJointPosition);
    }

    public void moveWinch(Gamepad gamepad2, OpMode OpModeInstance) {
        moveJoint(gamepad2.right_stick_y, OpModeInstance);
        int d = calcWinchEncoderValue(currentJointPosition, OpModeInstance);

        OpModeInstance.telemetry.addData("encoder value ", winchMotor.getCurrentPosition());

        runToPosition(d, OpModeInstance);

        OpModeInstance.telemetry.addData("d ", d);
    }

    public int calcWinchEncoderValue(double jointValue, OpMode OpModeInstance) {
        double degree = jointValue * 180;
        double r = Math.toRadians(degree / 2);

        double w = Math.sin(r) * l;

        double distance = 2 * w;

        double e = (-12000 / 34.25 ) * distance;

        return (int) e;
    }

    public void setJointValue(OpMode OpModeInstance) {
        int eValue = winchMotor.getCurrentPosition();

        double d = (34.25 * eValue) / 12000;

        double a = Math.asin((d/2) / l);

        double r = a * 2;

        double t = Math.toDegrees(r);

        jointServo.setPosition(t / 180);
    }

    public void adjustWinch(Gamepad gamepad2, OpMode opModeInstance) {

        if(gamepad2.right_bumper)
            winchMotor.setPower(.05);
        else if(gamepad2.left_bumper)
            winchMotor.setPower(-.05);
        else
            winchMotor.setPower(0);

    }

    private void runToPosition(int eVal, OpMode OpModeInstance){
        int p = winchMotor.getCurrentPosition();

        if(p > eVal){
            winchMotor.setPower(-.1);
        }
        else if(p < eVal)
            winchMotor.setPower(.1);
        else
            winchMotor.setPower(0);

        OpModeInstance.telemetry.addData("p ",p);
        OpModeInstance.telemetry.addData("eVal ", eVal);
    }
}
