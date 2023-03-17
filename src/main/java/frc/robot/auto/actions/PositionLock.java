package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.submodules.Chassis;

//position PID to lock position when on charged station
public class PositionLock implements Action{

    private final double kp = 0;
    private final double ki = 0;
    private final double kd = 0;
    private double lVel,rVel;
    private double lPos,rPos;
    private double speedMulti = 0.01;
    private double prevErrorL,prevErrorR;
    private double totalErrorL,totalErrorR;

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void update() {
        double lError = (Chassis.getInstance().getPeriodicIO().leftPosition-lPos)*speedMulti;
        double rError = (Chassis.getInstance().getPeriodicIO().rightPosition-rPos)*speedMulti;
        lVel = lError*kp+(lError-prevErrorL)*kd;
        prevErrorL = lError;
        rVel = rError*kp+(rError-prevErrorR)*kd;
        prevErrorR = rError;
        Chassis.getInstance().setPercentSpeed(lVel, rVel);
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void initialize() {
        lPos = Chassis.getInstance().getPeriodicIO().leftPosition;
        rPos = Chassis.getInstance().getPeriodicIO().rightPosition;
        totalErrorL = 0;
        totalErrorR=0;
        prevErrorL=0;
        prevErrorR=0;
    }
    
}
