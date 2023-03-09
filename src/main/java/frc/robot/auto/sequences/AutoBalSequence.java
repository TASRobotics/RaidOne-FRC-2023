package frc.robot.auto.sequences;

import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.AutoBalPID;

import java.util.Arrays;

public class AutoBalSequence extends AutoSequence {
    private static final String name = "AutoBalSequence";


    public AutoBalSequence() {
        
    }

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new AutoBalPID())
            )
        );
    }

    @Override
    public void onEnded() {
        System.out.println(name + " ended!");
    }

    @Override
    public String getName() {
        return name;
    }
}
