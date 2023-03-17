package frc.robot.auto.sequences;

import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.AutoBal;
import frc.robot.auto.actions.PositionLock;

import java.util.Arrays;

public class PositionLockSequence extends AutoSequence {
    private static final String name = "PositionLockSequence";


    public PositionLockSequence() {
        
    }

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new PositionLock()
                )
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
