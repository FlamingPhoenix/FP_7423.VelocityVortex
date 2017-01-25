package FlamingPhoenix;

import android.webkit.JavascriptInterface;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Created by Steve on 11/12/2016.
 */

public class MyMotor implements DcMotor {

    public DcMotor motor;

    private double currentPower;
    private long benchMarkTime;
    private int benchMarkEncoderTick;
    private RunMode myMode;
    private int encoderSpeed;

    public MyMotor(DcMotor originalMotor) {
        this.motor = originalMotor;
        this.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);

        currentPower = 0;
        myMode = RunMode.RUN_WITHOUT_ENCODER;
        encoderSpeed = 0;
    }


    @Override
    public void setMaxSpeed(int encoderTicksPerSecond) {
        encoderSpeed = Math.abs(encoderTicksPerSecond);
    }

    @Override
    public int getMaxSpeed() {
        return this.encoderSpeed;
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Deprecated
    @Override
    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        if (this.motor != null) {
            if (mode == RunMode.STOP_AND_RESET_ENCODER)
                this.motor.setMode(mode);
            else if ((mode == RunMode.RUN_WITHOUT_ENCODER) || (mode == RunMode.RUN_USING_ENCODER))
                this.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
            else
                this.motor.setMode(mode);
        }

        myMode = mode;
    }

    @Override
    public RunMode getMode() {
        return myMode;
    }

    @Override
    public void setDirection(Direction direction) {
        motor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    @Override
    public void setPower(double power) {
        if ((myMode == RunMode.RUN_WITHOUT_ENCODER) || (myMode == RunMode.RUN_TO_POSITION))
            motor.setPower(power);
        else if ((myMode == RunMode.STOP_AND_RESET_ENCODER) || (power == 0)) {
            motor.setPower(0);
            currentPower = 0;
        }
        else {
            int targetSpeed = (encoderSpeed * (int) Math.round(power * 1000)) / 1000;
            DbgLog.msg("[Phoenix:MyMotor.setPower] targetSpeed=%d", targetSpeed);

            if ((currentPower == 0) && (power != 0))  {
                benchMarkTime = System.currentTimeMillis();
                benchMarkEncoderTick = this.motor.getCurrentPosition();
            } else if (benchMarkEncoderTick > 2100000000) { //need to prevent integer overflow
                this.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
                this.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
                benchMarkTime = System.currentTimeMillis();
                benchMarkEncoderTick = this.motor.getCurrentPosition();
            }

            double powerIncrement= 0.01;
            if (power < 0)
                powerIncrement = -0.01;

            try {
                if (targetSpeed <= 500)
                    currentPower = 0.1;
                if (targetSpeed > 500 && targetSpeed < 900)
                    currentPower = 0.3;
                else if (targetSpeed >= 900)
                    currentPower = 0.5;

                this.motor.setPower(currentPower);
                Thread.sleep(1000);

                benchMarkTime = System.currentTimeMillis();
                benchMarkEncoderTick = this.motor.getCurrentPosition();

                int speed = 0;
                while (speed < (targetSpeed - 50))
                {
                    double motorPower = currentPower + powerIncrement;
                    if (motorPower > 1)
                        motorPower = 1;
                    else if (motorPower < -1)
                        motorPower = -1;

                    this.motor.setPower(motorPower);
                    currentPower = motorPower;

                    if (Math.abs(currentPower) == 1)
                        return;  //already reached maximum power;

                    Thread.sleep(100);
                    int currentTick = this.motor.getCurrentPosition();
                    speed = Math.round(Math.abs((((long) (currentTick- benchMarkEncoderTick)) * 1000) / (System.currentTimeMillis() - benchMarkTime)));
                    DbgLog.msg("[Phoenix.MyMotor.setPower] Speed = %d, power = %8.5f, curentTicket=%d", speed, currentPower, currentTick);
                }
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

        }
    }

    @Override
    public double getPower() {
        return currentPower;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {
        motor.close();
    }
}
