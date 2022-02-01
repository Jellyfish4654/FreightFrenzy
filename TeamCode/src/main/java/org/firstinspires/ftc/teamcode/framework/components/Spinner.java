package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.CRServo;
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
}