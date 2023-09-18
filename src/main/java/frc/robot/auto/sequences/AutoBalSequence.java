package frc.robot.auto.sequences;

import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.AutoBal;
import frc.robot.auto.actions.LambdaAction;
import frc.robot.submodules.Chassis;

import java.util.Arrays;

public class AutoBalSequence extends AutoSequence {
    private static final String name = "AutoBalSequence";


    public AutoBalSequence() {
        
    }

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                new AutoBal(),
                new LambdaAction(() -> Chassis.getInstance().smartHold())
                )
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
