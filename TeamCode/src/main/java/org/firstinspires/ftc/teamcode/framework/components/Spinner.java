package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class Spinner {
    private CRServo servo;

    public Spinner(CRServo servo) {
        this.servo = servo;
    }

    public void on() {
        servo.setPower(1);
    }
    
    public void off(){
        servo.setPower(0);
    }

    public void rev() {
        servo.setPower(-1);
    }

    public Task spin(DcMotorSimple.Direction direction) {
        double dir = direction == DcMotorSimple.Direction.FORWARD ? 1 : -1;
        // TODO...
    }
}