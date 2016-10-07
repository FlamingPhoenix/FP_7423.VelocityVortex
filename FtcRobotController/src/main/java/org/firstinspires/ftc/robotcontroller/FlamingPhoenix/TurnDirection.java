package org.firstinspires.ftc.robotcontroller.FlamingPhoenix;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

//Chassis
public enum TurnDirection {
    RIGHT,
    RIGHT_FORWARD,
    LEFT,
    LEFT_FORWARD,
}