package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Claw {
    private static final double GRAB_POSITION = 1.0;
    private static final double UNGRAB_POSITION = 0.5;
    private static final double SPEED = 0.15;   

    private DcMotor pivot;
    private Servo servo;

    public Claw(DcMotor pivot, Servo servo) {
        this.pivot=pivot;
        this.servo=servo;
    }

    public void up() {
        pivot.setPower(SPEED);
    }

    public void down() {
        pivot.setPower(-SPEED);
    }

    public void off() {
        pivot.setPower(0);
    }

    public void grab() {
        servo.setPosition(GRAB_POSITION);
    }

    public void ungrab() {
        servo.setPosition(UNGRAB_POSITION);
    }
}