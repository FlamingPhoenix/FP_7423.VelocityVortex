package org.firstinspires.ftc.robotcontroller.FlamingPhoenix;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class MecanumDriveTrain {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public MecanumDriveTrain(DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight) {
        frontLeft = FrontLeft;
        frontRight = FrontRight;
        backLeft = BackLeft;
        backRight = BackRight;
    }

    public void Drive(Gamepad gamePad, OpMode opModeInstance) {
        mecanumDrive(-gamePad.left_stick_x, gamePad.left_stick_y, gamePad.right_stick_x);
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
