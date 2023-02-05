package frc.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.actions.DrivePath;
import frc.robot.auto.actions.LambdaAction;
import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.AutoBal;
import java.util.List;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class leftFront extends AutoSequence {
    private static final Trajectory path1 = PathPlanner.loadPath("leftFront", AutoConstants.MAX_VEL, AutoConstants.MAX_ACCEL);
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
