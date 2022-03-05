package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.framework.Task;

public class Intake {
    DcMotorSimple motor;
    public Intake(DcMotorSimple motor) {
        this.motor = motor;
    }

    public void input() {
        motor.setPower(1);
    }

    public void output() {
        motor.setPower(-1);
    }

    public void stop() {
        motor.setPower(0);
    }
}
