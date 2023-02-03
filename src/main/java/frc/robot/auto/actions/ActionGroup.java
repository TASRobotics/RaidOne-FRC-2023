package frc.robot.auto.actions;

import java.util.ArrayList;
import java.util.List;

/**
 * Action for running multiple Actions in a series
 */
public class ActionGroup implements Action{

    private Action currentAction;
    private final ArrayList<Action> remainingActions;

    public ActionGroup(List<Action> actions){
        remainingActions = new ArrayList<>(actions);
        currentAction = null;
    }

    @Override
    public boolean isDone(){
        return remainingActions.isEmpty() && currentAction == null;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void update(){
        if(currentAction == null){
            if(remainingActions.isEmpty()){
                return;
            }
            currentAction = remainingActions.remove(0);
            currentAction.initialize();
        }

        currentAction.update();

        if(currentAction.isDone()){
            currentAction.done();
            currentAction = null;
        }
    }

    @Override
    public void done(){
        if(currentAction != null){
            currentAction.done();
        }
    }
}
