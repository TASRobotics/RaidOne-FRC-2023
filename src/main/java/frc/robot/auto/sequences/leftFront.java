package frc.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.actions.DrivePath;
import frc.robot.auto.actions.ActionGroup;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class leftFront extends AutoSequence {
    private static final Trajectory path1 = PathPlanner.loadPath("Left", 1.5, AutoConstants.MAX_ACCEL);
    public leftFront() {}

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new DrivePath(path1, true)
            )
        ));
    }

    @Override
    public void onEnded() {
        System.out.println("leftFront ended!");
    }

    @Override
    public String getName() {
        return "leftFront";
    }
}
