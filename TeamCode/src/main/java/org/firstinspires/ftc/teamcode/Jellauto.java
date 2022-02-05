package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.Task;
import org.firstinspires.ftc.teamcode.framework.components.Auto;
import org.firstinspires.ftc.teamcode.framework.components.Vision;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

@Autonomous(name = "Jellauto")
public class Jellauto extends BaseOpMode {
    protected Auto dt;

    protected static enum Team { RED, BLUE }
    protected static enum Position { WAREHOUSE, CAROUSEL }
    protected static enum Scoring { L1, L2, L3 }

    protected Team team = Team.BLUE;
    protected Position position = Position.CAROUSEL;
    protected Scoring scoring;
    
    protected Vision vision;

    protected boolean hasElement() throws InterruptedException {
        final BlockingQueue<Vision.Result<int[]>> profileQueue = new LinkedBlockingQueue();
        vision.getColorProfile((profile) -> {
            try {
                profileQueue.put(profile);
            } catch (Exception e) {}
            return null;
        });
        Vision.Result<int[]> res = profileQueue.take();
        if (res.error != null) {
            throw new RuntimeException("vision error [1]: " + res.error);
        }
        return containsElement(res.value);
    }

    protected static boolean containsElement(int[] profile) {
        return profile[1] >= 100 && profile[2] >= 20 && profile[5] >= 20;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(); 
        claw.grab();
        dt = new Auto(motors, imu);
        vision = new Vision("Webcam 1", hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.x) team = Team.BLUE;
            if (gamepad1.b) team = Team.RED;
            if (gamepad1.dpad_up) position = Position.WAREHOUSE;
            if (gamepad1.dpad_down) position = Position.CAROUSEL;

            telemetry.addData("team", team);
            telemetry.addData("position", position);
            telemetry.update();
        }
        waitForStart();

        // move claw up
        if (Task.run(claw.up(180), this)) return;

        if (hasElement()) {
            /// center = L2 /// (appendix D)
            scoring = Scoring.L2;
            if (Task.run(dt.move(5, 0.5), this)) return;
            dt.stop();
        } else {
            /// left = L1 ///
            if (Task.run(dt.move(5, 0.5), this)) return;

            // pivot 30 degrees counterclockwise (left)
            if (Task.run(dt.pivot(30, 0.5), this)) return;


            if (hasElement()) {
                scoring = Scoring.L1;
            } else {
            /// right = L3 ///
                scoring = Scoring.L3;
            }

            if (Task.run(dt.pivot(-30, 0.5), this)) return;
        }

        telemetry.addData("team", team);
        telemetry.addData("position", position);
        telemetry.addData("scoring", scoring);
        telemetry.update();

        if(Task.run(
            Task.seq(
                dt.move(24+8, 0.5),
                () -> {
                    telemetry.addData("done moving", ":)");
                    telemetry.update();
                    return true;
                },
                dt.pivot((
                    (team == Team.RED && position == Position.WAREHOUSE) || (team == Team.BLUE && position == Position.CAROUSEL) ? // is the hub on the left?
                    -90 : // if so, turn right
                    90
                    ), 0.5),
                () -> {
                    telemetry.addData("done pivoting", ":)");
                    telemetry.update();
                    return true;
                }
            )
        , this)) return;

        int off = 0;
        if (scoring == Scoring.L2) {
            if (Task.run(dt.move(2, 0.5), this)) return;
            off = 2;
        } else if (scoring == Scoring.L1) {
            // might not work..  pivot...?
            if (Task.run(dt.move(4, 0.5), this)) return;
            off = 4;
        }
        claw.ungrab(); // release!!!!!!!!!!

        telemetry.addData("done scoring", "");
        telemetry.update();

        int reverseRed = team == Team.RED ? -1 : 1; // "for blue"
        if (position == Position.CAROUSEL) {            
            if (Task.run(Task.seq(
                dt.move(20 - off, 0.5),
                dt.pivot(-90 * reverseRed, 0.5),
                dt.move(24, 0.5),
                Task.wait(5000, () -> { spinner.on(); return false; }),
                () -> { spinner.off(); return true; },
                dt.move(-24, 0.5)
            ), this)) return;

            // :)
        } else {
            if (Task.run(Task.seq(
                dt.move(4 - off, 0.5),  
                dt.pivot(90 * reverseRed, 0.5),
                dt.move(4, 0.5),
                dt.pivot(90 * reverseRed, 0.5),
                dt.move(36, 1)
            ), this)) return;

            // :) parked
        }
    }
}
