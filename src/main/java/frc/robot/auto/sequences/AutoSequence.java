package frc.robot.auto.sequences;

import java.util.LinkedList;
import java.util.Queue;

import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.ActionGroup;

public abstract class AutoSequence {
    protected boolean isSequenceRunning = false;

    protected Queue<Action> actions = new LinkedList<>();
    protected Action currentAction = null;

    /**
     * Should provide a sequence of {@link Action}s using 
     * {@link #addAction(Action)} to execute during autonomous.
     */
    protected abstract void sequence();

    /**
     * Runs at the start of the sequence.
     * 
     * Note: This should never be called by user code, only by 
     * {@link AutoRunner}.
     */
    public final void run() {
        // Reset the actions
        actions.clear();
        currentAction = null;

        sequence();

        // End the sequence if there aren't any actions
        if (actions.isEmpty()) {
            onEnded();
            return;
        }
        isSequenceRunning = true;
    }

    /**
     * Overridable method that is called at the end of the sequence.
     */
    public void onEnded() {
        System.out.println("[Auto] Auto sequence '" + getName() + "' ended!");
    }

    /**
     * Stops the autonomous sequence without calling {@link #onEnded()}
     */
    public void stop() {
        isSequenceRunning = false;
    }

    /**
     * Whether the sequence is currently running.
     * 
     * @return if the sequence is running
     */
    public boolean isRunning() {
        return isSequenceRunning;
    }

    /**
     * Adds an action to the action queue.
     * 
     * @param seriesAction action to add
     */
    public void addAction(ActionGroup seriesAction) {
        actions.add(seriesAction);
    }

    /**
     * Executes actions from the action queue.
     * 
     * Note: This should never be called by user code, only by
     * {@link AutoRunner}.
     * 
     * @param timestamp
     */
    public void onLoop(double timestamp) {
        // Don't run actions if the sequence is not running
        if (!isSequenceRunning) {
            return;
        }
        // Start the next action if we're done with the previous
        if (currentAction == null) {
            currentAction = actions.poll();
            currentAction.initialize();
        }
        currentAction.update();

        // Get ready to start the next action
        if (currentAction.isDone()) {
            currentAction.done();
            currentAction = null;

            // End the sequence if we're done with all actions
            if (actions.isEmpty()) {
                onEnded();
                stop();
                return;
            }
        }
    }

    /**
     * Returns a user-friendly name for the autonomous sequence.
     * 
     * @return a user-friendly name
     */
    public String getName() {
        return "Nameless Sequence";
    }

    @Override
    public String toString() {
        return getName();
    }
}
