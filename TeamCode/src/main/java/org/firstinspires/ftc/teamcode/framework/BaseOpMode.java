package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.Motors;
import org.firstinspires.ftc.teamcode.framework.components.Spinner;
import org.firstinspires.ftc.teamcode.framework.components.Claw;

public abstract class BaseOpMode extends LinearOpMode {
    protected DcMotor[] motors;
    protected Claw claw;
    protected Spinner spinner;
    protected void initHardware() {
        motors = new DcMotor[] {
            hardwareMap.dcMotor.get("motor fr"),
            hardwareMap.dcMotor.get("motor br"),
            hardwareMap.dcMotor.get("motor fl"),
            hardwareMap.dcMotor.get("motor bl")
        };

        motors[Motors.FR].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[Motors.BR].setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo carousel = hardwareMap.crservo.get("carousel");
        spinner = new Spinner(carousel);

        DcMotor clawPivot = hardwareMap.dcMotor.get("claw-pivot");
        Servo clawServo = hardwareMap.servo.get("claw-servo");
        claw = new Claw(clawPivot, clawServo);
    }
}