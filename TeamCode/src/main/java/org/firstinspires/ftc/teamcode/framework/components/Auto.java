package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.framework.Task;
import org.firstinspires.ftc.teamcode.framework.Motors;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;

public class Auto {

    // measurements
    // 1000 ticks / 19 in
    // 500 ticks / 9.5 in
    public static final double ENCODERS_PER_IN = 500.0 / 9.5; // experimental testing needed

    protected DcMotor[] motors;
    protected BNO055IMU imu;
    public Auto(DcMotor[] motors, BNO055IMU imu) {
        this.motors = motors;
        this.imu = imu;
    }

    public Task move(double distance, double angle, double maxSpeed) {
        return new MoveTask(this.motors, distance, angle, maxSpeed);
    }

    private static class PivotState {
        boolean initialized;
        double targetAngle;
        double initialAngle;
    }

    // angle in degrees
    public Task pivot(double angle, double maxSpeed) {
        final PivotState state = new PivotState();
        state.initialized = false;

        final double direction = angle > 0 ? 1 : -1;

        final double P = 1.0 / 10.0; // at an angle of 10 and above, P*angle >= 1
        // 0.2 power <=> 2 deg *P

        return () -> {
            if (!state.initialized) {
                state.initialAngle = ((imu.getAngularOrientation().firstAngle % 360) + 360) % 360;
                state.targetAngle = (state.initialAngle + angle + 360) % 360;
                for (DcMotor motor : motors) {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                state.initialized = true;
            }

            double currentAngle = ((imu.getAngularOrientation().firstAngle % 360) + 360) % 360;
            double diff = (state.targetAngle - currentAngle + 360) % 360; // positive number, [0, 360]

            if (diff <= 2 || Math.abs(diff-360) <= 2) {
                stop();
                return true;
            }

            double pow;
            if (direction == 1) {
                pow = diff * P;
            } else { // direction == -1
                pow = (diff-360)*P;
            }
            if (pow > maxSpeed) pow = maxSpeed;
            if (pow < -maxSpeed) pow = -maxSpeed;

            BaseOpMode.tele.addData("initial", state.initialAngle);
            BaseOpMode.tele.addData("target", state.targetAngle);
            BaseOpMode.tele.addData("current", currentAngle);
            BaseOpMode.tele.addData("diff", diff);
            BaseOpMode.tele.addData("pow", pow);
            BaseOpMode.tele.update();

            motors[Motors.FR].setPower(pow);
            motors[Motors.BR].setPower(pow);
            motors[Motors.FL].setPower(-pow);
            motors[Motors.BL].setPower(-pow);
            return false;
        };
    }

    public void stop() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
    }
}

class MoveTask implements Task {
    private double encoderDist;
    private double angle;
    private double maxSpeed;

    DcMotor[] motors;
    public MoveTask(DcMotor[] motors, double distance, double angle, double maxSpeed) {
        this.encoderDist = distance * Auto.ENCODERS_PER_IN;
        this.angle = angle;
        this.maxSpeed = maxSpeed;
        this.motors = motors;
    }

    private static double encoderAverage(double value1, double value2) {
        if (value1 == 0) return value2;
        if (value2 == 0) return value1;
        return  (value1+ value2)/2;
    }

    private final static double KP = 0.1;
    private final static double KD = 0;

    private boolean initialized;
    private double previousDist;
    private long previousTime;
    public boolean step() {
        if (!initialized) {
            for (DcMotor motor: motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            initialized = true;
        }

        // a = [1,1]
        // b = [-1,1]
        double aDistTarget = encoderDist * Math.cos((angle + 45) * Math.PI / 180);
        double bDistTarget = encoderDist * Math.sin((angle + 45) * Math.PI / 180);

        double aDistCurrent = encoderAverage(motors[Motors.BL].getCurrentPosition(), motors[Motors.FR].getCurrentPosition());
        double bDistCurrent = encoderAverage(motors[Motors.FL].getCurrentPosition(), motors[Motors.BR].getCurrentPosition());

        double dist = Math.sqrt((aDistCurrent - aDistTarget)*(aDistCurrent - aDistTarget) + (bDistCurrent - bDistTarget)*(bDistCurrent - bDistTarget)) * Math.signum(aDistTarget - aDistCurrent);
        if (dist < 0.15 * Auto.ENCODERS_PER_IN) {
            return true;
        }

        double pow;
        if (previousTime == 0) {
            pow = dist*KP;
        } else {
            long time = System.currentTimeMillis();
            pow = dist*KP + (dist-previousDist)/(time-previousTime)*KD;
            previousDist = dist;
            previousTime = time;
        }

        double aVel = pow * Math.cos((angle + 45) * Math.PI / 180);
        double bVel = pow * Math.sin((angle + 45) * Math.PI / 180);
        motors[Motors.BL].setPower(aVel);
        motors[Motors.FR].setPower(aVel);
        motors[Motors.BR].setPower(bVel);
        motors[Motors.FL].setPower(bVel);
        return false;
    }
}