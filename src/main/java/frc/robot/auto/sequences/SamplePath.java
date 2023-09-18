package frc.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.actions.MoveTime;
import frc.robot.auto.actions.autobaltest;
import frc.robot.auto.actions.ActionGroup;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class SamplePath extends AutoSequence {
    private static final String name = "forward";

    private static final Trajectory path1 = PathPlanner.loadPath(name, 1.5, AutoConstants.MAX_ACCEL);

    public SamplePath() {
        
    }

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                //DrivePath(path1, true))
                new MoveTime(-0.1, 1),
                new autobaltest()
            )
        ));
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
