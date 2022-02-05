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

            final BlockingQueue<Vision.Result<int[]>> queue = new LinkedBlockingQueue();
            vision.findAreas2((areas) -> {
                try {
                    queue.put(areas);
                } catch(Exception e) {}
                return null;
            });

            Vision.Result<int[]> res = queue.take();
            if (res.error != null) {
                telemetry.addData("error", res.error);
            } else {
                telemetry.addData("success", Arrays.toString(res.value));
                boolean found = false;
                if (res.value[1] >= 100 && res.value[2] >= 20 && res.value[5] >= 20) {
                    found = true;
                }
                telemetry.addData("found", found);
            }
            telemetry.update();

            if (Task.run(Task.wait(10000, null), this)) return;
//        }
    }
}