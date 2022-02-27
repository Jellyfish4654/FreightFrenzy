package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.Task;
import org.firstinspires.ftc.teamcode.framework.Motors;
import org.firstinspires.ftc.teamcode.framework.components.Auto;
import org.firstinspires.ftc.teamcode.framework.components.Vision;

import java.lang.Thread;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.List;
import java.util.Arrays;

import android.graphics.Rect;

@TeleOp(name = "Vision Test")
public class VisionTest extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("init", "initializing camera...");
        telemetry.update();
        Vision vision = new Vision("Webcam 1", hardwareMap);
        telemetry.addData("init", "initialized camera");
        telemetry.update();
        waitForStart();

//        while(opModeIsActive()) {
            telemetry.addData("start", "detecting...");
            telemetry.update();

            final BlockingQueue<Vision.Pixels> queue = new LinkedBlockingQueue();
            vision.pixels((pixels) -> {
                try {
                    queue.put(pixels);
                } catch(Exception e) {}
                return null;
            });

            Vision.Pixels pixels = queue.take();
            double[] profile = pixels.colorProfile(new Rect(
                pixels.width / 2 - pixels.width / 8, // left
                pixels.height * 3/4 - pixels.height / 8, // top
                pixels.width / 2 + pixels.width / 8, // right
                pixels.height * 3/4 + pixels.height / 8 // bottom
            ));
            telemetry.addData("profile", profile);
            telemetry.update();

            if (Task.run(Task.wait(10000, null), this)) return;
//        }
    }
}