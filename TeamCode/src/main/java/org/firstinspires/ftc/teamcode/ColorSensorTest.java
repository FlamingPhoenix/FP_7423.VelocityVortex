package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.*;

/**
 * Created by HwaA1 on 11/16/2016.
 */

@TeleOp(name = "colortest", group = "none")

public class ColorSensorTest extends OpMode {

    ColorSensor color;
    ModernRoboticsI2cGyro gyro;

    @Override
    public void init() {
        color = hardwareMap.colorSensor.get("color");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
    }

    @Override
    public void loop() {


        int red = color.red();
        int blue = color.blue();
        I2cAddr addr = color.getI2cAddress();

        telemetry.addData("red", red);
        telemetry.addData("blue", blue);
        telemetry.addData("gyro", gyro.getHeading());

        telemetry.update();
    }
}
