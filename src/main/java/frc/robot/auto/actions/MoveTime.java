package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.submodules.Chassis;

public class MoveTime implements Action{

    private double initialTime;
    private double vel, time;

    /**
     * 
     * @param vel -1 to 1. forward positive, backwards neg
     * @param time moving time in seconds
     */
    public MoveTime(double vel, double time){
        this.vel = vel;
        this.time = time;
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        if(Timer.getFPGATimestamp()>initialTime+time){
            return true;
        }
        return false;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        if(Timer.getFPGATimestamp()<initialTime+time){
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
        initialTime = Timer.getFPGATimestamp();
    }
    
}
