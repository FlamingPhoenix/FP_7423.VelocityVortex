package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by HwaA1 on 10/20/2016.
 */

public class ShootingTest extends OpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    private DcMotor shooter;

    @Override
    public void init() {
        backLeft = hardwareMap.dcMotor.get("backleft");
        backRight = hardwareMap.dcMotor.get("backright");
        frontLeft = hardwareMap.dcMotor.get("frontleft");
        frontRight = hardwareMap.dcMotor.get("frontright");
        shooter = hardwareMap.dcMotor.get("shooter");

        //Left Side motors should rotate opposite of right side motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.y)
            shooter.setPower(70);
        else if(gamepad1.b)
            shooter.setPower(60);
        else if (gamepad1.a)
            shooter.setPower(50);
        else if(gamepad1.x)
            shooter.setPower(25);
    }
}
