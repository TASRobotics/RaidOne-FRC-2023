package frc.robot.auto.actions;

import frc.robot.submodules.Chassis;

public class AutoBal implements Action{

    public boolean isDone(){
        //change this!
        return false;
    }

    public void update(){
        if (Chassis.getInstance().getPeriodicIO().pitch >= 5) {
                
        } else if (Chassis.getInstance().getPeriodicIO().pitch <= -5) {
                
        }
        return;
    }

    public void initialize(){

    }

    public void done(){

    }
    
}
