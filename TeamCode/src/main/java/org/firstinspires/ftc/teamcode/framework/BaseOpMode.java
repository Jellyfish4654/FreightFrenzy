package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.framework.Motors;
import org.firstinspires.ftc.teamcode.framework.components.Spinner;
import org.firstinspires.ftc.teamcode.framework.components.Claw;


public abstract class BaseOpMode extends LinearOpMode {
    public static Telemetry tele;

    protected DcMotor[] motors;
    protected Claw claw;
    protected Spinner spinner;
    protected BNO055IMU imu;
    protected void initHardware() {
        BaseOpMode.tele = telemetry;

        motors = new DcMotor[] {
            hardwareMap.dcMotor.get("motor fr"),
            hardwareMap.dcMotor.get("motor br"),
            hardwareMap.dcMotor.get("motor fl"),
            hardwareMap.dcMotor.get("motor bl")
        };

        motors[Motors.FL].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[Motors.BL].setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo carousel = hardwareMap.crservo.get("carousel");
        spinner = new Spinner(carousel);

        DcMotor clawPivot = hardwareMap.dcMotor.get("claw-pivot");
        Servo clawServo = hardwareMap.servo.get("claw-servo");
        claw = new Claw(clawPivot, clawServo);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); 
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit  = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "imu";
        imu.initialize(parameters);        
    }
}