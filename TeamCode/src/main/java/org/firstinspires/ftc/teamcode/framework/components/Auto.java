package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.framework.Task;
import org.firstinspires.ftc.teamcode.framework.Motors;

public class Auto {
    private static final double ENCODERS_PER_IN = 1.0; // experimental testing needed

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
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    motor.setTargetPosition(motor.getCurrentPosition() + (int)(ENCODERS_PER_IN * distance));
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

    // angle in degrees
    public Task pivot(double angle, double maxSpeed) {
        final double direction = angle > 0 ? 1 : -1;
        final double initialAngle = ((imu.getAngularOrientation().firstAngle % 360) + 360) % 360;

        final double P = 1 / 15; // at an angle of 15 and above, P*angle >= 1

        return () -> {
            double currentAngle = ((imu.getAngularOrientation().firstAngle % 360) + 360) % 360;
            double diff = (currentAngle - initialAngle + 360) % 360; // positive number, [0, 360]

            if (Math.abs(diff) <= 1) {
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

            motors[Motors.FR].setPower(pow);
            motors[Motors.FL].setPower(pow);
            motors[Motors.BR].setPower(-pow);
            motors[Motors.BL].setPower(-pow);
            return false;
        };
    }
}