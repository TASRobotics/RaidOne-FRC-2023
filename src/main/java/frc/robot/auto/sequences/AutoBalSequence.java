package frc.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.actions.DrivePath;
import frc.robot.auto.actions.LambdaAction;
import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.AutoBal;
import frc.robot.auto.actions.AutoBalPID;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class AutoBalSequence extends AutoSequence {
    private static final String name = "AutoBalSequence";


    public AutoBalSequence() {
        
    }

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new AutoBalPID())
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
