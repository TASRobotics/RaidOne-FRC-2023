package frc.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.actions.DrivePath;
import frc.robot.auto.actions.LambdaAction;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.AutoBal;
import frc.robot.auto.actions.MoveDistance;
import frc.robot.auto.actions.MoveTime;
import frc.robot.submodules.Chassis;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class Left extends AutoSequence {
    private static final Trajectory path1 = PathPlanner.loadPath("Left", 1.8, AutoConstants.MAX_ACCEL);
    private static final Trajectory path2 = PathPlanner.loadPath("Left cone", 1.5, AutoConstants.MAX_ACCEL,true);
    //private static final autobaltest autobal = new autobaltest();
    public Left() {}

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new DrivePath(path2,true),
                //new MoveTime(0.3,0.1),
                new DrivePath(path1),
                new AutoBal(),
                new LambdaAction(() -> Chassis.getInstance().smartHold())
                //new DrivePath(path1),
                //autobal
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
