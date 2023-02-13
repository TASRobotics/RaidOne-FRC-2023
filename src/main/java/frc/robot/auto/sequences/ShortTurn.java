package frc.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.actions.DrivePath;
import frc.robot.auto.actions.LambdaAction;
import frc.robot.auto.actions.ActionGroup;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class ShortTurn extends AutoSequence {
    private static final String name = "ShortTurn";

    private static final Trajectory path1 = PathPlanner.loadPath(name, AutoConstants.MAX_VEL, AutoConstants.MAX_ACCEL);

    public ShortTurn() {
        
    }

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new DrivePath(path1, true))
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
