package org.firstinspires.ftc.robotcontroller.FlamingPhoenix;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Steve on 9/24/2016.
 */

public class ProgrammingBot extends OpMode {

    MecanumDriveTrain DriveTrain;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    @Override
    public void init() {
        frontLeft = this.hardwareMap.dcMotor.get("frontleft");
        backLeft = this.hardwareMap.dcMotor.get("backleft");
        frontRight = this.hardwareMap.dcMotor.get("frontright");
        backRight = this.hardwareMap.dcMotor.get("backright");

        DriveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void loop() {
        DriveTrain.Drive(this.gamepad1, this);
        telemetry.addData("x1 ", gamepad1.left_stick_x);
        telemetry.addData("y1 ", gamepad1.left_stick_y);
        telemetry.addData("x2 ", gamepad1.right_stick_x);
    }
}
