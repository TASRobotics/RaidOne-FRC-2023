package frc.robot.auto.sequences;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.DrivePath;
import frc.robot.auto.actions.RamBack;

public class ConeDrop extends AutoSequence {

    public ConeDrop() {

    }

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new RamBack()
                )
                //new DrivePath(path1, true))
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
