package frc.robot.auto.sequences;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.DrivePath;
import frc.robot.auto.actions.MoveDistance;

public class ConeDrop extends AutoSequence {

    public ConeDrop() {

    }

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new MoveDistance(0.1,0.5)
                )
            )
        );
        
    }

    @Override
    public void onEnded() {
        
    }

    @Override
    public String getName() {
        return "ConeDrop";
    }
}
