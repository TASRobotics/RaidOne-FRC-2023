package frc.robot.auto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.sequences.*;

/**
 * class that manages auton sequences
 */
public class AutoRunner {

    private SendableChooser<AutoSequence> chooser;

    private AutoSequence selectedSequence;

    //all auto sequences here
    private AutoSequence[] sequences = {
        new Left(),
        new Right(),
        new Middle(),
        new ShortTurn(),
        new ConeDrop(),
        new PositionLockSequence(),
        new AutoBalSequence(),
        new SamplePath()
    };

    public AutoRunner(){
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("none", null);
        for(AutoSequence sequence: sequences){
            chooser.addOption(sequence.getName(), sequence);
        }
        Shuffleboard.getTab("MAIN")
                .add("Auton Selection", chooser)
                .withSize(3, 1)
                .withPosition(2, 1);
    }

    /**
     * Reads the selected autonomous sequence from the SendableChooser.
     */
    public void readSendableSequence() {
        selectSequence(chooser.getSelected());
    }

    /**
     * Selects an autonomous sequence to run.
     * 
     * @param sequence the autonomous sequence to run
     */
    public void selectSequence(AutoSequence sequence) {
        selectedSequence = sequence;
    }

    /**
     * Returns the currently selected autonomous sequence.
     * 
     * @return the selected sequence - null means no sequence.
     */
    public AutoSequence getSelectedSequence() {
        return selectedSequence;
    }

    /**
     * Starts the selected autonomous sequence.
     */
    public void start() {
        if (selectedSequence != null) {
            System.out.println("[Auto] Starting auto sequence '" + selectedSequence.getName() + "'...");
            selectedSequence.run();
        }
    }

    /**
     * Stops the selected autonomous sequence.
     */
    public void stop() {
        if (selectedSequence != null) {
            System.out.println("[Auto] Stopping auto sequence '" + selectedSequence.getName() + "'...");
            selectedSequence.stop();
        }
    }

    /**
     * Updates the selected autonomous sequence.
     * 
     * @param timestamp
     */
    public void onLoop(double timestamp) {
        if (selectedSequence != null) {
            selectedSequence.onLoop(timestamp);
        }
    }
}
