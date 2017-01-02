package org.firstinspires.ftc.teamcode;

import FlamingPhoenix.ServoArm;
import FlamingPhoenix.StealthDriveTrain;
import FlamingPhoenix.TapeMeasure;
import FlamingPhoenix.Winch;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;
//import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Steve on 10/11/2015.
 */
@TeleOp(name="ResQTeleOp", group="Old Program")
@Disabled()
public class ResQTeleOp extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor winchMotor;
    private DeviceInterfaceModule cdim;
    private ServoController servoCont;
    private DcMotor sweep;

    static final int LED_CHANNEL = 5;

    private Servo positionServo;
    private Servo frontPlateRightServo;
    private Servo frontPlateLeftServo;
    private Servo leftHookServo;
    private Servo rightHookServo;
    private Servo rotationServo;

    private StealthDriveTrain sDriveTrain;
    private TapeMeasure winch;
    double leftHookServoPosition = .5;
    double rightHookServoPosition = .5;

    private Boolean setWinchEncoder;

    private ServoArm FrontArm;

    @Override
    public void init() {
        hardwareMap.logDevices();

        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");


        sDriveTrain = new StealthDriveTrain(leftMotor, rightMotor);

        frontPlateLeftServo = hardwareMap.servo.get("fpls");
        frontPlateLeftServo.setPosition(1);

        frontPlateRightServo = hardwareMap.servo.get("fprs");
        frontPlateRightServo.setPosition(0);

        leftHookServo = hardwareMap.servo.get("lhs");
        leftHookServo.setPosition(0); //point up as part of initialization step

        rightHookServo = hardwareMap.servo.get("rhs");
        rightHookServo.setPosition(1); //point up as part of initialization step

        winchMotor = hardwareMap.dcMotor.get("winch");
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWinchEncoder = true;

        positionServo = hardwareMap.servo.get("servop");
        positionServo.setPosition(0.5);


        winch = new TapeMeasure(winchMotor, positionServo);

        FrontArm = new ServoArm(hardwareMap.servo.get("servobase"), hardwareMap.servo.get("servojoint"), hardwareMap.servo.get("servorotate"));
    }

    @Override
    public void loop() {

        movePanels(gamepad1);
        moveHooks(gamepad2);

        if (setWinchEncoder) {
            winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setWinchEncoder = false;
        } else {
            winch.moveWinch(gamepad2, this);
        }

        sDriveTrain.JoystickDrive(gamepad1, gamepad2);
        FrontArm.operateByRemote(gamepad1, this); //move all-clear arm
    }

    public void moveHooks(Gamepad gamepad1) {
        double e = 0.01;

        if (gamepad1.left_bumper) {
            double r = rightHookServo.getPosition() + e;
            if (r <= 1) {
                rightHookServo.setPosition(r);
                double l = 1.0 - r;
                leftHookServo.setPosition(l);
            }
        }
        else if (gamepad1.left_trigger > .5) {
            double l = leftHookServo.getPosition() + e;
            if (l <= 1) {
                leftHookServo.setPosition(l);
                double r = 1.0 - l;
                rightHookServo.setPosition(r);
            }
        }
    }

    public void movePanels(Gamepad gamepad2) {
        double e = 0.01;
        double l = frontPlateLeftServo.getPosition();
        double r = frontPlateRightServo.getPosition();

        if (gamepad2.left_bumper) {
            l = frontPlateLeftServo.getPosition() + e;
            if(l <= 1.0) {
                frontPlateLeftServo.setPosition(l);
            }
        } else if (gamepad2.left_trigger > .5) {
            l = frontPlateLeftServo.getPosition() - e;
            if(l >= 0)
                frontPlateLeftServo.setPosition(l);
        }

        if (gamepad2.right_bumper){
            r = frontPlateRightServo.getPosition() - e;
            if(r >= 0) {
                frontPlateRightServo.setPosition(r);
            }
        } else if (gamepad2.right_trigger > .5) {
            r = frontPlateRightServo.getPosition() + e;
            if(r <= 1) {
                frontPlateRightServo.setPosition(r);
            }
        }
    }

    public void sweep(Gamepad gamepad1){
        if (gamepad1.left_stick_y > 0.1 || gamepad1.right_stick_y > 0.1 ) {
            sweep.setPower(-1.0);
        }
        else
            sweep.setPower(0);
    }
}
