package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.framework.Task;
import org.firstinspires.ftc.teamcode.framework.Motors;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;

public class Auto {
    protected DcMotor[] motors;
    public Auto(DcMotor[] motors) {
        this.motors = motors;
    }

    protected Pose pose;
    public Pose position() {
        return new PositioningTask(motors, this.pose);
    }

    public static class Pose {
        public Pose(double x, double y, double angle) {
            this.x = x;
            this.y = y;
            this.angle = angle;
        }

        public Pose clone() {
            return new Pose(x, y, angle);
        }

        public double x;
        public double y;
        public double angle;

        /** Forward offset; positive if horizontal encoder in front, negative if in back */
        private final static double F = -1;

        /** The lateral distance, distance between the two vertical encoders */
        private final static double L = 1;

        public void applyDiff(double d_left, double d_right, double d_horiz) {
            double phi = (d_left - d_right) / L;
            double dxc = (d_left + d_right) / 2,
                dxh = d_horiz - (F * phi);

            double[] diff = calculateDiff(dxc, dxh, phi, angle);
            x += diff[0];
            y += diff[1];
            angle += phi;
        }

        /** Given dxc (forward/back), dxh (horizontal), and phi (angle), return dx, dy */
        private static double[] calculateDiff(double dxc, double dxh, double phi, double prev_angle) {
            // i think i did this wrong
            // need to double check
            double dxc2 = dxc * Math.sin(phi)/phi + dxh * ((Math.cos(phi) - 1) / phi),
                dxh2 = dxc * ((1 - Math.cos(phi)) / phi) + dxh * Math.sin(phi)/phi;

            double dx = dxc2 * Math.cos(prev_angle) + dxh2 * -Math.sin(prev_angle),
                dy = dxc2 * Math.sin(prev_angle) + dxh2 * Math.cos(prev_angle);

            return new double[] {dx, dy};
        }
    }
}

class PositioningTask implements Task {
    protected DcMotor[] motors;
    protected Auto.Pose pose_;

    public PositioningTask(DcMotor[] motors, Auto.Pose poseToUpdate) {
        this.motors = motors;
        pose_ = poseToUpdate;
    }

    /** Previous encoder positions */
    protected double[] prevEncoderPosition = new double[3];
    protected double[] currEncoderPosition = new double[3];
    public boolean step() {
        for (int i = 0; i < 3; i++) {
            currEncoderPosition[i] = motors[i].getCurrentPosition();
        }

        pose_.applyDiff(currEncoderPosition[Motors.E_L] - prevEncoderPosition[Motors.E_L],
            currEncoderPosition[Motors.E_R] - prevEncoderPosition[Motors.E_R],
            currEncoderPosition[Motors.E_H] - prevEncoderPosition[Motors.E_H]);

        for (int i = 0; i < 3; i++) {
            prevEncoderPosition[i] = currEncoderPosition[i];
        }
        return false;
    }
}


/* OLD AUTO CLASS
/** Represents a drivetrain, for use during autonomous *
public class Auto {
    // measurements
    // 1000 ticks / 19 in
    // 500 ticks / 9.5 in
    // new
    // 500 ticks / 14 in

    public static final double ENCODERS_PER_IN = 500 / 14; // experimental testing needed

    protected DcMotor[] motors;
    protected BNO055IMU imu;
    public Auto(DcMotor[] motors, BNO055IMU imu) {
        this.motors = motors;
        this.imu = imu;
    }

    // positive angle = right
    public Task move(double distance, double angle, double maxSpeed) {
        return new MoveTask(motors, imu, distance, angle, maxSpeed);
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

    protected DcMotor[] motors;
    protected BNO055IMU imu;

    public MoveTask(DcMotor[] motors, BNO055IMU imu, double distance, double angle, double maxSpeed) {
        this.encoderDist = distance * Auto.ENCODERS_PER_IN;
        this.angle = angle;
        this.maxSpeed = maxSpeed;
        this.motors = motors;
        this.imu = imu;
    }

    private static double encoderAverage(double value1, double value2) {
        if (value1 == 0) return value2;
        if (value2 == 0) return value1;
        return (value1 + value2)/2;
    }

    private final static double KP = 0.003;
    private final static double KD = 0.001;

    private boolean initialized;
    private double previousDist;
    private long previousTime;
    private double currentAngle;
    private double targetAngle;
    public boolean step() {
        if (!initialized) {
            for (DcMotor motor: motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            targetAngle = ((imu.getAngularOrientation().firstAngle % 360) + 360) % 360;

            initialized = true;
        }

        // a = [1,1]
        // b = [-1,1]
        double aDistTarget = encoderDist * Math.cos((angle + 45) * Math.PI / 180);
        double bDistTarget = encoderDist * Math.sin((angle + 45) * Math.PI / 180);

        double aDistCurrent = encoderAverage(motors[Motors.BL].getCurrentPosition(), motors[Motors.FR].getCurrentPosition());
        double bDistCurrent = encoderAverage(motors[Motors.FL].getCurrentPosition(), motors[Motors.BR].getCurrentPosition());

        BaseOpMode.tele.addData("target", aDistTarget);
        BaseOpMode.tele.addData("current", aDistCurrent);
        BaseOpMode.tele.update();

        double dist = Math.sqrt((aDistCurrent - aDistTarget)*(aDistCurrent - aDistTarget) + (bDistCurrent - bDistTarget)*(bDistCurrent - bDistTarget)) * Math.signum(aDistTarget*(aDistTarget - aDistCurrent));
        if (Math.abs(dist) < 1 * Auto.ENCODERS_PER_IN) {
            for (DcMotor motor: motors) {
                motor.setPower(0);
            }
            return true;
        }

        // calculate distance in angle
        double currentAngle = ((imu.getAngularOrientation().firstAngle % 360) + 360) % 360;
        double angleDiff = (targetAngle - currentAngle + 360) % 360; // positive number, [0, 360]
        if (angleDiff > 180) {
            angleDiff -= 360;
        }
        double anglePow = angleDiff * 0.001;

        // calculate power
        double pow;
        if (previousTime == 0) {
            pow = dist*KP;
        } else {
            long time = System.currentTimeMillis();
            pow = dist*KP + (dist-previousDist)/(time-previousTime)*KD;
            previousDist = dist;
            previousTime = time;
        }

        if (pow > maxSpeed) {
            pow = maxSpeed;
        }
        if (pow < -maxSpeed) {
            pow = -maxSpeed;
        }

        double aVel = pow * Math.cos((angle + 45) * Math.PI / 180);
        double bVel = pow * Math.sin((angle + 45) * Math.PI / 180);
        motors[Motors.BL].setPower(aVel - anglePow);
        motors[Motors.FR].setPower(aVel + anglePow);
        motors[Motors.BR].setPower(bVel + anglePow);
        motors[Motors.FL].setPower(bVel - anglePow);
        return false;
    }
} */