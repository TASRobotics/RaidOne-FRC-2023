package frc.robot.auto.actions;

import frc.robot.submodules.Chassis;

public class VelControlTest implements Action{

    private double initialEncoderL;

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        if(Chassis.getInstance().getPeriodicIO().leftPosition<initialEncoderL+1){
            Chassis.getInstance().setPercentSpeed(0.1, 0.1);
            System.out.println("moving");
        }
        else{
            Chassis.getInstance().setPercentSpeed(0, 0);
            System.out.println("freeze");
        }
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void initialize() {

        //should be converted to meters
        initialEncoderL = Chassis.getInstance().getPeriodicIO().leftPosition; 
    }
    
}
