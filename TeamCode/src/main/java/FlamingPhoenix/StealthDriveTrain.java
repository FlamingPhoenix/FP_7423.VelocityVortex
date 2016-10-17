package FlamingPhoenix;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Austin_2 on 11/21/2015.
 */
public class StealthDriveTrain {

    private int ppr;
    private float wheelDiameter;
    private float wheelCircumference;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public StealthDriveTrain(DcMotor left, DcMotor right) {
        leftMotor = left;
        rightMotor = right;

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        ppr = 1680;
        wheelDiameter = 6.0f;
        wheelCircumference = (float)((double)wheelDiameter * Math.PI);
    }

    public void JoystickDrive(Gamepad Gamepad1, Gamepad gamepad2) {
        if (Gamepad1 != null) {

            float leftPower = Range.clip(Gamepad1.left_stick_y, -1, 1);
            float rightPower = Range.clip(Gamepad1.right_stick_y, -1, 1);

            if (((Gamepad1.left_stick_y > 0.5) && (Gamepad1.right_stick_y < -0.5)) || ((Gamepad1.left_stick_y < -0.5) && (Gamepad1.right_stick_y > 0.5))) {
                    leftPower = leftPower * 0.7f;
                    rightPower = rightPower * 0.7f;
            }

            leftPower = (float) scaleInput(leftPower);
            rightPower = (float) scaleInput(rightPower);
            if(!gamepad2.b) {
                leftMotor.setPower(leftPower * .7);
                rightMotor.setPower(rightPower * .7);
            }
            else {
                leftMotor.setPower(leftPower);
                rightMotor.setPower(rightPower);
            }
        }
    }

    public void moveArm(Gamepad gamepad2, Servo s1, Servo s2){
        if(gamepad2.left_stick_y > 0);
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
