package frc.robot.auto.sequences;

import java.util.Arrays;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.DrivePath;

public class ConeDrop extends AutoSequence {

    private static final String name = "Cone forward";

    private static final Trajectory path1 = PathPlanner.loadPath(name, AutoConstants.MAX_VEL, AutoConstants.MAX_ACCEL, true);

    public ConeDrop() {}


    
    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new DrivePath(path1, true)
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
