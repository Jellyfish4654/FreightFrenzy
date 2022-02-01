package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** This class represents a method that will be called repeatedly in a loop. */
public interface Task {
    /**
     * Runs one step of a task. This should not be called after the task is complete.
     * @return true if the task is complete, false otherwise
     */
    boolean step();

    /**
     * Runs an entire task, repeatedly calling its step function.
     * @return true if the op has ended and the surrounding function should return
     */
    public static boolean run(Task task, LinearOpMode opMode) {
        while (true) {
            if (opMode != null && !opMode.opModeIsActive()) {
                return true;
            }
            if (task.step()) {
                return false;
            }
        }
    }

    /**
     * Combines multiple tasks into one task that completes
     * when all of its tasks complete.
     */
    public static Task all(Task... tasks) {
        return new AllTask(tasks);
    }
}

// Class that is actually used in the implementation of the Task.all method
class AllTask implements Task {
    Task[] tasks;
    boolean[] complete;

    public AllTask(Task[] tasks) {
        this.tasks = tasks;
        complete = new boolean[tasks.length];
    }

    public boolean step() {
        // whether all of the tasks are complete
        boolean allComplete = true;
        for (int i = 0; i < tasks.length; i++) {
            if (!complete[i]) {
                // if a task is not complete, run a step
                complete[i] = tasks[i].step();

                allComplete = false;
            }
        }

        return allComplete;
    }
}