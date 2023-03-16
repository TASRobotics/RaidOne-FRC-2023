package frc.robot.auto.actions;

import frc.robot.submodules.Chassis;

public class MoveDistance implements Action{

    private double initialX;
    private double vel, dist;

    /**
     * 
     * @param vel -1 to 1. forward positive, backwards neg
     * @param dist positive number, meters
     */
    public MoveDistance(double vel, double dist){
        this.vel = vel;
        this.dist = dist;
        if(vel<0&&dist>0) dist*=-1;
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        if(Chassis.getInstance().getPeriodicIO().x<initialX+dist&&dist>0){
            Chassis.getInstance().setPercentSpeed(vel, vel);
            //System.out.println("moving");
        }
        else if(Chassis.getInstance().getPeriodicIO().x>initialX+dist&&dist<0){
            Chassis.getInstance().setPercentSpeed(vel,vel);
        }
        else{
            Chassis.getInstance().setPercentSpeed(0, 0);
            done();
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
        initialX = Chassis.getInstance().getPeriodicIO().x;
    }
    
}
