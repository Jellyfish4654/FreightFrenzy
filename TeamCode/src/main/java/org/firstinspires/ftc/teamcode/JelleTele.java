package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.Motors;
import org.firstinspires.ftc.teamcode.framework.components.Spinner;
import org.firstinspires.ftc.teamcode.framework.components.Claw;

@TeleOp(name = "UltimateGoal JelleTele")
public class JelleTele extends LinearOpMode {
    protected static enum DriveMode {
        TANK,
        DRIVE,
        MECANUM,
    }

    protected DriveMode driveMode = DriveMode.MECANUM;

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

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        initHardware();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                driveMode = DriveMode.TANK;
            } else if (gamepad1.dpad_up) {
                driveMode = DriveMode.MECANUM;
            } else if (gamepad1.dpad_right) {
                driveMode = DriveMode.DRIVE;
            }

            
            double mult = gamepad1.left_bumper ? 0.2 : gamepad1.right_bumper ? 0.5 : 1.0;

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

            if (gamepad2.y) {
                spinner.on(DcMotorSimple.Direction.FORWARD);
            } else if (gamepad2.x){
                spinner.on(DcMotorSimple.Direction.REVERSE);
            } else {
                spinner.off();
            }

            if (gamepad2.right_bumper) {
                claw.grab();
            } else {
                claw.ungrab();
            }

            if (gamepad2.left_stick_y > 0) {
                claw.up();
            } else if (gamepad2.left_stick_y < 0) {
                claw.down();
            } else {
                claw.off();
            }


//            logger.update();
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

//        logger.addData("powers", "%.2f %.2f %.2f %.2f", powers[0], powers[1], powers[2], powers[3]);
/*
        for (int i = 0; i < 4; i++) {
            powers[i] *= 1.0;
        }*/

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }
    }
}
