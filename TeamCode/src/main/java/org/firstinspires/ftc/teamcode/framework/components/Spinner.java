package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.framework.Task;
public class Spinner {
    private DcMotor motor;

    public Spinner(DcMotor motor) {
        this.motor = motor;
    }

    public void on() {
        motor.setPower(0.2);
    }
    
    public void off(){
        motor.setPower(0);
    }

    public void rev() {
        motor.setPower(-0.2);
    }

    public Task spin(DcMotorSimple.Direction direction) {
        double dir = direction == DcMotorSimple.Direction.FORWARD ? 1 : -1;
        // TODO...
        return null;
    }
}