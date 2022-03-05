/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.Task;
import org.firstinspires.ftc.teamcode.framework.Motors;
import org.firstinspires.ftc.teamcode.framework.components.Auto;
import org.firstinspires.ftc.teamcode.framework.components.Vision;

import java.io.File;
import android.os.Environment;
import java.io.FileOutputStream;
import java.io.OutputStream;

@Autonomous(name = "Odometry Tuning :)")
public class OdometryTuning extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        double initialAngle = imu.getAngularOrientation().firstAngle;

        motors[Motors.FL].setPower(-0.2);
        motors[Motors.BL].setPower(-0.2);
        motors[Motors.FR].setPower(0.2);
        motors[Motors.BR].setPower(0.2);

        while (opModeIsActive()) {
            double currAngle = imu.getAngularOrientation().firstAngle;
            double diffAngle = currAngle - initialAngle;

            long leftEnc = motors[Motors.E_L].getCurrentPosition(),
                rightEnc = -motors[Motors.E_R].getCurrentPosition(),
                horizEnc = motors[Motors.E_H].getCurrentPosition();

            telemetry.addData("angle diff", diffAngle);
            telemetry.addData("data", String.format("L:%d R:%d H:%d", leftEnc, rightEnc, horizEnc));

            if (diffAngle < 0 && diffAngle > -90) {
                for (DcMotor motor : motors) {
                    motor.setPower(0);
                }

                double L = (leftEnc - rightEnc) / Math.toRadians(diffAngle + 360);
                double F = (horizEnc) / Math.toRadians(diffAngle + 360);

                telemetry.addData("constants", String.format("L:%.2f F:%.2f", L, F));
                telemetry.update();

                try {
                    // write data to file
                    File dir = new File(Environment.getExternalStorageDirectory(), ".freight-frenzy");
                    dir.mkdirs();
                    OutputStream file = new FileOutputStream(new File(dir, "odo.txt"));
                    file.write(String.format("L %f\nF %f\n", L, F).getBytes());
                } catch(Exception e) {
                    throw new RuntimeException(e.getMessage());
                }

                break;
            }

            telemetry.update();
        }

        Task.run(Task.wait(10*1000,null), this);
    }
}*/