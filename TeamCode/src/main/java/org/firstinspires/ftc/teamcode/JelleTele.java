
//Jeffrey was here C:
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.Motors;
import org.firstinspires.ftc.teamcode.framework.Task;
import org.firstinspires.ftc.teamcode.framework.components.Spinner;
import org.firstinspires.ftc.teamcode.framework.components.Auto;
import org.firstinspires.ftc.teamcode.framework.components.Arm;

@TeleOp(name = "Freight Frenzy JelleTele")
public class JelleTele extends BaseOpMode {
    protected static enum DriveMode {
        TANK,
        DRIVE,
        MECANUM,
    }

    protected DriveMode driveMode = DriveMode.MECANUM;

    protected Task spinnerTask = null;
    protected Task armTask = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        auto.pose = new Auto.Pose(0, 0, 0);
        Task positionTask = auto.position();
        while (opModeIsActive()) {
            positionTask.step();

            if (gamepad1.dpad_left) {
                driveMode = DriveMode.TANK;
            } else if (gamepad1.dpad_up) {
                driveMode = DriveMode.MECANUM;
            } else if (gamepad1.dpad_right) {
                driveMode = DriveMode.DRIVE;
            }

            double mult = gamepad1.left_bumper ? 0.35 : gamepad1.right_bumper ? 0.7 : 1.0;

            telemetry.addData("drive mode", driveMode);
            telemetry.addData("precision mode", mult);
            telemetry.addData("pose", auto.pose.toString());
            telemetry.addData("encoders", auto.debugEncoders());
            telemetry.addData("arm", String.format("%d %d", arm.position()[0], arm.position()[1]));
            telemetry.update();

            switch (driveMode) {
            case TANK: {
                double l = -gamepad1.left_stick_y,
                    r = -gamepad1.right_stick_y;
                setMotorSpeeds(mult, new double[] {r, r, l, l});
                break;
            }
            case DRIVE: {
                double pivot = gamepad1.left_stick_x, y = -gamepad1.left_stick_y;
                setMotorSpeeds(mult, new double[] {
                    y-pivot,
                    y-pivot,
                    y+pivot,
                    y+pivot
                });
                break;
            }
            case MECANUM: {
                // right = +, left = -
                double pivot = gamepad1.right_stick_x;
                double mX, mY;
                mX = gamepad1.left_stick_x;
                mY = -gamepad1.left_stick_y;
                setMotorSpeeds(mult, new double[] {
                    mY - mX - pivot,
                    mY + mX - pivot,
                    mY + mX + pivot,
                    mY - mX + pivot});
                break;
            }
            }

            if(gamepad1.right_trigger >0.5) {
                for (DcMotor motor: motors) {
                    motor.setPower(0);
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else {
                for (DcMotor motor: motors){
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }

            if (gamepad2.x) { // blue
                spinnerTask = spinner.run(DcMotorSimple.Direction.REVERSE);
            } else if (gamepad2.b) { // red
                spinnerTask = spinner.run(DcMotorSimple.Direction.FORWARD);
            } else if (gamepad2.a) {
                spinner.stop();
                spinnerTask = null;
            } else {
                if (spinnerTask != null) {
                    boolean completed = spinnerTask.step();
                    if (completed) {
                        spinnerTask = null;
                    }
                }
            }

            // normal arm movement
            arm.move(gamepad2.left_stick_y * (
                (gamepad2.right_trigger != 0 ? 0.2 : 1)));
            if (gamepad2.left_stick_y != 0) {
                armTask = null;
            }

            // automatic arm controls [dont work]
            if (gamepad2.dpad_up) {
                armTask = arm.moveTo(Arm.POSITION_HIGH);
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                armTask = arm.moveTo(Arm.POSITION_MID);
            } else if (gamepad2.dpad_down) {
                armTask = arm.moveTo(Arm.POSITION_LOW);
            }
            if (armTask != null) armTask.step();
            if (gamepad2.left_bumper) {
                arm.setBaseline();
            }

            if (gamepad2.right_stick_y > 0) {
                intake.input();
            } else if (gamepad2.right_stick_y < 0) {
                intake.output();
            } else {
                intake.stop();
            }

            if (gamepad2.right_bumper) {
                intake.doorOpen();
            } else { 
                intake.doorClose();
            }
        }
    }

    /**
     * Corrects the given motor powers so that they are all <= 1.0
     * and sets the motor powers.
     */
    protected void setMotorSpeeds(double mult, double[] powers) {
        for (int i = 0; i < 4; i++) {
            powers[i] = powers[i] * mult;
        }

        double max = Math.max(Math.max(Math.abs(powers[0]), Math.abs(powers[1])), Math.max(Math.abs(powers[2]), Math.abs(powers[3])));
        double scale = Math.abs(1 / max);
        // don't increase power, only decrease
        if (scale > 1) {
            scale = 1;
        }

        for (int i = 0; i < 4; i++) {
            powers[i] *= scale;
        }

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }
    }
}
