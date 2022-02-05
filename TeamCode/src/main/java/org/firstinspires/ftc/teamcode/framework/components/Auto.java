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
    private static final double ENCODERS_PER_IN = 500.0 / 9.5; // experimental testing needed

    protected DcMotor[] motors;
    protected BNO055IMU imu;
    public Auto(DcMotor[] motors, BNO055IMU imu) {
        this.motors = motors;
        this.imu = imu;
    }

    private static class MoveState {
        boolean setTarget;
        int[] startPosition;
    }
    
    public Task move(double distance, double maxSpeed) {
        // workaround because ban on mutable local variables
        final MoveState s = new MoveState();
        s.setTarget = false;
        s.startPosition = new int[4];

        return () -> {
            if (!s.setTarget) {
                // this section only run once
                s.setTarget = true;

                for (int i = 0; i < 4; i++) {
                    DcMotor motor = motors[i];

                    motor.setTargetPosition(motor.getCurrentPosition() + (int)(ENCODERS_PER_IN * distance));
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(maxSpeed);
                    s.startPosition[i] = motor.getCurrentPosition();
                }
                return false;
            } else {
                // return true (complete) only if none of the motors are running
                return !motors[0].isBusy() && !motors[1].isBusy() && !motors[2].isBusy() && !motors[3].isBusy();
            }
        };
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