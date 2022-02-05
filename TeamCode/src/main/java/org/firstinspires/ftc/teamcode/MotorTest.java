package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.Task;
import org.firstinspires.ftc.teamcode.framework.Motors;
import org.firstinspires.ftc.teamcode.framework.components.Auto;

@TeleOp(name = "MotorTest")
public class MotorTest extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();        
        waitForStart();
        
        while (opModeIsActive()) {
            if (gamepad1.a) {
                motors[Motors.FL].setPower(0.5);
            } else {
                motors[Motors.FL].setPower(0);
            }

            if (gamepad1.b) {
                motors[Motors.FR].setPower(0.5);
            } else {
                motors[Motors.FR].setPower(0);
            }


            if (gamepad1.x) {
                motors[Motors.BL].setPower(0.5);
            } else {
                motors[Motors.BL].setPower(0);
            }


            if (gamepad1.y) {
                motors[Motors.BR].setPower(0.5);
            } else {
                motors[Motors.BR].setPower(0);
            }

        }
    }
}
