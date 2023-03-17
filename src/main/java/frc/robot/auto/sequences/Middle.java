package frc.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.actions.DrivePath;
import frc.robot.auto.actions.autobaltest;
import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.AutoBal;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class Middle extends AutoSequence {
    private static final Trajectory path1 = PathPlanner.loadPath("forward", 2, AutoConstants.MAX_ACCEL);
    private static final Trajectory path2 = PathPlanner.loadPath("Cone forward", 1.5, AutoConstants.MAX_ACCEL, true);
    public Middle() {}

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                // new DrivePath(path2, true),
                // new DrivePath(path1),
                new autobaltest()
            )
        ));
    }

    @Override
    public void onEnded() {
        System.out.println("Middle ended!");
    }

    @Override
    public String getName() {
        return "Middle";
    }
}
