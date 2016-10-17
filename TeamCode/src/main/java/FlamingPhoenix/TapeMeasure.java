package FlamingPhoenix;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;

import java.sql.Time;

/**
 * Created by Austin_2 on 12/6/2015.
 */
public class TapeMeasure {
    private DcMotor winchMotor;
    private Servo positionServo;
    private final double winchMinimum = 0.0;
    private final double winchMaximum = 28400;

    public TapeMeasure(DcMotor winch, Servo servo){
        winchMotor = winch;
        positionServo = servo;
    }

    public void moveWinch(Gamepad gamepad2,OpMode OpModeInstance) {
        double jVal = -gamepad2.right_stick_y;

        if(gamepad2.right_stick_y < -0.1 &&  Math.abs(gamepad2.left_stick_y) < 0.1) {
            if (winchMotor.getCurrentPosition() < winchMaximum)
                winchMotor.setPower(jVal);
            else if (gamepad2.right_bumper)
                winchMotor.setPower(jVal);
            else
                winchMotor.setPower(0);
        }
        else if (gamepad2.right_stick_y > 0.1  &&  Math.abs(gamepad2.left_stick_y) < 0.1) {
            if (winchMotor.getCurrentPosition() > winchMinimum)
                winchMotor.setPower(jVal);
            else if (gamepad2.right_bumper)
                winchMotor.setPower(jVal);
            else
                winchMotor.setPower(0);
        }
        else {
            winchMotor.setPower(0);
        }

        //Control the continuous servo
        if (gamepad2.left_stick_y < -.1 && Math.abs(gamepad2.right_stick_y) < 0.1) {
            positionServo.setPosition(0.6); //0.6 means going forward at 0.1 power
        } else if (gamepad2.left_stick_y > .1 && Math.abs(gamepad2.right_stick_y) < 0.1) {
            positionServo.setPosition(0.4); //0.4 means reverse at 0.1 power
        } else {
            positionServo.setPosition(0.5); //0.5 means no power
        }

        OpModeInstance.telemetry.addData("joyStickValue", jVal);
        OpModeInstance.telemetry.addData("WinchMotPosit", winchMotor.getCurrentPosition());
        OpModeInstance.telemetry.update();
    }
}
