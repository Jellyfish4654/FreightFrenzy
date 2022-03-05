package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.framework.Task;

public class Intake {
    DcMotorSimple motor;
    Servo door;

    static final double DOOR_OPEN = 1;
    static final double DOOR_CLOSE = 0;

    public Intake(DcMotorSimple motor, Servo door) {
        this.motor = motor;
        this.door = door;
    }
    
    public void doorOpen() {
        door.setPosition(DOOR_OPEN);
    }

    public void doorClose() {
        door.setPosition(DOOR_CLOSE);

    }

    public void input() {
        motor.setPower(-1);
    }

    public void output() {
        motor.setPower(1);
    }

    public void stop() {
        motor.setPower(0);
    }
}
