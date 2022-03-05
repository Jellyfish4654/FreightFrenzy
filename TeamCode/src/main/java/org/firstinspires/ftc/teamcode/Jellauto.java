package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.Task;
import org.firstinspires.ftc.teamcode.framework.components.Auto;
import org.firstinspires.ftc.teamcode.framework.components.Vision;

import java.util.concurrent.BlockingQueue;  
import java.util.concurrent.LinkedBlockingQueue;

@Autonomous(name = "Freight Frenzy Jellauto")
public class Jellauto extends BaseOpMode {
    protected Auto dt;

    protected static enum Team { RED, BLUE }
    protected static enum Position { WAREHOUSE, CAROUSEL }
    protected static enum Algorithm { TEST, SIMPLE, VISION }
//    protected static enum Scoring { L1, L2, L3 }

    protected Team team = Team.BLUE;
    protected Position position = Position.CAROUSEL;
    protected Algorithm algorithm = Algorithm.SIMPLE;
//    protected Scoring scoring;
    
//    protected Vision vision;
/*
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
    }*/

    protected Task wrapPosition(Task inner) {
        return Task.sim(
            auto.position(),
            () -> {
                telemetry.addData("pose", auto.pose);
                telemetry.update();
                return false;
            },
            inner
        );
    }

    final static double Pm = 0.007;
    final static double Pa = 1.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(); 
//        vision = new Vision("Webcam 1", hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.x) team = Team.BLUE;
            if (gamepad1.b) team = Team.RED;
            if (gamepad1.left_bumper) position = Position.WAREHOUSE;
            if (gamepad1.right_bumper) position = Position.CAROUSEL;
            if (gamepad1.dpad_down) algorithm = Algorithm.TEST;
            if (gamepad1.dpad_up) algorithm = Algorithm.SIMPLE;
            if (gamepad1.dpad_left) algorithm = Algorithm.VISION;

            telemetry.addData("team", team);
            telemetry.addData("position", position);
            telemetry.addData("algorithm", algorithm);
            telemetry.update();
        }
        waitForStart();

        final double REV_IF_RED = team == Team.RED ? -1 : 1;

        switch (algorithm) {
        case TEST:
            auto.pose = new Auto.Pose(0, 0, 0);
            Auto.Pose target = new Auto.Pose(0, 0, Math.PI / 2);
            Task.run(Task.sim(
                auto.position(),
                () -> {
                    telemetry.addData("pose", auto.pose);
                    telemetry.addData("target", target);
                    telemetry.update();
                    return false;
                },
                Task.seq(
                    auto.moveTo(target, 0.5, Pm, Pa)
                )
            ), this);
            break;
        case SIMPLE:
            if (position == Position.CAROUSEL) {
                auto.pose = new Auto.Pose(0,                                        24*Auto.ENC_PER_IN, -Math.PI / 2);
                Task.run(wrapPosition(Task.seq(
                    auto.moveTo(new Auto.Pose(0,                                    8.6 * Auto.ENC_PER_IN, -Math.PI / 2), 0.3, Pm/3, Pa),
                    spinner.run(team == Team.BLUE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD),
                    auto.moveTo(new Auto.Pose(REV_IF_RED * -36*Auto.ENC_PER_IN,     32*Auto.ENC_PER_IN, -Math.PI/2), 0.4, Pm, Pa),
                    () -> { intake.doorOpen(); return true; },
                    Task.wait(1000, null),
                    Task.wait(4000, () -> {intake.input(); return false; }),
                    () -> { intake.stop(); intake.doorClose(); return true; },
                    auto.moveTo(new Auto.Pose(REV_IF_RED * -29.5*Auto.ENC_PER_IN,   2 *Auto.ENC_PER_IN, -Math.PI/2), 0.5, Pm, Pa)
                )), this);
            } else { // position == WAREHOUSE
                auto.pose = new Auto.Pose(0, 6*Auto.ENC_PER_IN, -Math.PI / 2);
                Task.run(wrapPosition(Task.seq(
                    auto.moveTo(new Auto.Pose(REV_IF_RED * 40*Auto.ENC_PER_IN,      8.5 * Auto.ENC_PER_IN, -Math.PI/2), 0.5, Pm, Pa),
                    () -> { intake.doorOpen(); return true; },
                    Task.wait(1000, null),
                    Task.wait(4000, () -> {intake.input(); return false; }),
                    () -> { intake.stop(); intake.doorClose(); return true; },
                    auto.moveTo(new Auto.Pose(0, 4 * Auto.ENC_PER_IN, -Math.PI / 2), 0.5, Pm, Pa),
                    auto.moveTo(new Auto.Pose(0, -24 * Auto.ENC_PER_IN, -Math.PI / 2), 0.5, Pm, Pa)
                )), this);
            }
            break;
        }
    }
}
