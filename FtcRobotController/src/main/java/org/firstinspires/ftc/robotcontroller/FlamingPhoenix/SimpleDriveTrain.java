package org.firstinspires.ftc.robotcontroller.FlamingPhoenix;

/**
 * Created by Steve on 9/24/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SimpleDriveTrain extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;

    @Override
    public void init() {
        leftWheel = this.hardwareMap.dcMotor.get("LeftWheel");
        rightWheel = this.hardwareMap.dcMotor.get("RightWheel");
    }

    @Override
    public void loop() {
        leftWheel.setPower(gamepad1.left_stick_y);
        rightWheel.setPower(gamepad1.right_stick_y);
    }
}
