package frc.robot.auto.sequences;

import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.VelControlTest;

import java.util.Arrays;

public class VelControlTestSequence extends AutoSequence {
    private static final String name = "VelControlTestSequence";


    public VelControlTestSequence() {
        
    }

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new VelControlTest())
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
