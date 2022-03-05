package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.framework.Task;

public class Arm {
    DcMotor[] motors;
    private static final double MAX_SPEED = 0.5;

    public static final long[] POSITION_HIGH = new long[] {500,500};
    public static final long[] POSITION_MID = new long[]{700,700};
    public static final long[] POSITION_LOW = new long[]{1000,1000};

    public Arm(DcMotor[] motors) {
        this.motors = motors;

        for (DcMotor motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  
        }
    }

    public void setBaseline() {
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  
        }
    }

    public long[] position() {
        return new long[] {
            motors[0].getCurrentPosition(),
            motors[1].getCurrentPosition()
        };
    }                                                                   

    public void test(int n) {
        motors[(n+1)%2].setPower(0);
        motors[n].setPower(MAX_SPEED);
    }

    public void move(double speed) {
        for (DcMotor motor: motors) {
            motor.setPower(MAX_SPEED * speed);
        }
    }

    public void stop() {
        for (DcMotor motor: motors) {
            motor.setPower(0);
        }
    }

    public Task moveTo(long[] encoderPositions) {
        return new ToTask(encoderPositions);
    }

    private class ToTask implements Task {
        long[] encoderPositions;
        public ToTask(long[] encoderPositions) {
            this.encoderPositions = encoderPositions;
        }

        public boolean step() {
            for (int i = 0; i < 2; i++) {
                long curr = motors[i].getCurrentPosition();
                long target = encoderPositions[i];

                // u(t) = e(t) * P
                double pow = (target - curr) * 0.2;

                // maximum speed :D
                if (pow > MAX_SPEED) pow = MAX_SPEED;
                if (pow < -MAX_SPEED) pow = -MAX_SPEED;

                motors[i].setPower(pow);
            }
            return false;
        }
    }
}

