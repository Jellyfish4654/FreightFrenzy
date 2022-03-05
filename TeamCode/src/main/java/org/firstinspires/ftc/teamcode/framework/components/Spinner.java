package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.framework.Task;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;

public class Spinner {
    private DcMotor motor;

    public Spinner(DcMotor motor) {
        this.motor = motor;
    }
    
    public void stop(){
        motor.setPower(0);
    }

    public Task run(DcMotorSimple.Direction direction) {
        return new SpinnerTask(motor, direction);
    }
}

class SpinnerTask implements Task {
    DcMotor motor;
    double dir;
    public SpinnerTask(DcMotor motor, DcMotorSimple.Direction direction) {
        dir = direction == DcMotorSimple.Direction.FORWARD ? 1 : -1;
        this.motor = motor;
    }

    boolean initialized = false;
    public boolean step() {
        if (!initialized) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            initialized = true;
        }

        if (Math.abs(motor.getCurrentPosition()) < 1800) {
            motor.setPower((Math.abs(motor.getCurrentPosition() * (0.65 / 1800)) + 0.35) * dir); // faster?
        } else if (Math.abs(motor.getCurrentPosition()) < 1800+1050) {
            motor.setPower(1 * dir);
        } else {
            motor.setPower(0);
            return true;
        }

        return false;
    }
}