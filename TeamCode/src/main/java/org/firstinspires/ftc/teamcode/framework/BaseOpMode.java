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
import org.firstinspires.ftc.teamcode.framework.components.Intake;
import org.firstinspires.ftc.teamcode.framework.components.Arm;
import org.firstinspires.ftc.teamcode.framework.components.Auto;

public abstract class BaseOpMode extends LinearOpMode {
    public static Telemetry tele;

    protected DcMotor[] motors;
    protected Auto auto;
 
    protected Arm arm;
    protected Intake intake;
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

        motors[Motors.FR].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[Motors.FL].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[Motors.BL].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[Motors.BR].setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        DcMotor carousel = hardwareMap.dcMotor.get("carousel");
        spinner = new Spinner(carousel);

        DcMotor[] armMotors = new DcMotor[] {
            hardwareMap.dcMotor.get("arm l"),
            hardwareMap.dcMotor.get("arm r")
        };
        armMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        arm = new Arm(armMotors);

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");
        Servo intakeDoor = hardwareMap.servo.get("door");
        intake = new Intake(intakeMotor, intakeDoor);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); 
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit  = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "imu";
        imu.initialize(parameters);

        auto = new Auto(motors);
    }
}