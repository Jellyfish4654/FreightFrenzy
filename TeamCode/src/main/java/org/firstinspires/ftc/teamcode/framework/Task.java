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
     * Combines multiple tasks into one task that
     * executes all given tasks simultaneously
     * amd completes when all of the tasks complete.
     */
    public static Task sim(Task... tasks) {
        return new SimTask(tasks);
    }

    /**
     * Creates a new Task
     * that executes all given tasks sequentially.
     */
    public static Task seq(Task... tasks) {
        return new SeqTask(tasks);
    }

    /**
     * Creates a new Task that 
     * when executed executes the inner
     * Task for at most N milliseconds.
     */
    public static Task wait(long millis, Task inner) {
        final long start = System.currentTimeMillis();
        return () -> {
            if (System.currentTimeMillis() - start >= millis) {
                return true;
            }

            if (inner != null)
                return inner.step();
            else
                return false;
        };
    }
}

// Class that is actually used in the implementation of the Task.all method
class SimTask implements Task {
    Task[] tasks;
    boolean[] complete;

    public SimTask(Task[] tasks) {
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

class SeqTask implements Task {
    Task[] tasks;
    int idx;

    public SeqTask(Task[] tasks) {
        this.tasks =tasks;
        idx = 0;
    }

    public boolean step() {
        if (tasks[idx].step()) {
            // if task complete increase idx
            idx++;

            if (idx == tasks.length) { // no more tasks
                return true;
            }
        }

        return false;
    }
}