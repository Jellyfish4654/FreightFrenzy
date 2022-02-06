package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.Task;

public class Claw {
    private static final double GRAB_POSITION = 1.0;
    private static final double UNGRAB_POSITION = 0.5;
    private static final double SPEED = 0.25;  

    private static final double ENCODERS_PER_DEG = 8.5;

    private DcMotor pivot;
    private Servo servo;

    public Claw(DcMotor pivot, Servo servo) {
        this.pivot=pivot;
        this.servo=servo;
    }

    public void powerUp() {
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setPower(SPEED);
    }
    public void powerDown() {
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setPower(-SPEED);
    }
    public void powerOff() {
        pivot.setPower(0);
    }

    public void fix() {
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setPower(0);
    }

    private static class UpState {
        boolean initialized;
    }
    public Task up(double angle) {
        final UpState state = new UpState();
        state.initialized = false;
        return () -> {
            if (!state.initialized) {
                pivot.setPower(SPEED/1.5);
                pivot.setTargetPosition(pivot.getCurrentPosition() + (int)(ENCODERS_PER_DEG * angle));
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setPower(SPEED/1.5);
                state.initialized = true;
                return false;
            }

            return !pivot.isBusy();
        };
    }

    public Task down(double angle) {
        return up(-angle);
    }

    public void grab() {
        servo.setPosition(GRAB_POSITION);
    }

    public void ungrab() {
        servo.setPosition(UNGRAB_POSITION);
    }
}