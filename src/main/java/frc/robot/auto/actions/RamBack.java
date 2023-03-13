package frc.robot.auto.actions;

import frc.robot.submodules.Chassis;

//rams backwards for .5 meter  
public class RamBack implements Action{

    private double initialEncoderL;

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        if(Chassis.getInstance().getPeriodicIO().x<initialEncoderL+0.5){
            Chassis.getInstance().setPercentSpeed(0.1, 0.1);
            //System.out.println("moving");
        }
        else{
            Chassis.getInstance().setPercentSpeed(0, 0);
            //System.out.println("freeze");
        }
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void initialize() {

        //should be converted to meters
        initialEncoderL = Chassis.getInstance().getPeriodicIO().x;
    }
    
}
