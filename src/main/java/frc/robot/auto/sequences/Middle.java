package frc.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.actions.DrivePath;
import frc.robot.auto.actions.LambdaAction;
import frc.robot.submodules.Chassis;
import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.AutoBal;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class Middle extends AutoSequence {
    private static final Trajectory path1 = PathPlanner.loadPath("forward", 1.5, AutoConstants.MAX_ACCEL,true);
    private static final Trajectory path2 = PathPlanner.loadPath("Cone forward", 2, AutoConstants.MAX_ACCEL);
    public Middle() {}

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                 new DrivePath(path2, true),
                 new DrivePath(path1),
                 new AutoBal(),
                 new LambdaAction(() -> Chassis.getInstance().smartHold())
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
