package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class Spinner {
    // Make two mehods: 
    //  - on(DcMotorSimple.Direction), which turns the spinner on
    //  - off()

    private CRServo servo;


    public Spinner(CRServo servo) {
        this.servo = servo;
    }

    // ...

    public void on(DcMotorSimple.Direction direction) {
        servo.setDirection(direction);
        servo.setPower(1);
    }
    
    public void off(){
        servo.setPower(0);
    }
}