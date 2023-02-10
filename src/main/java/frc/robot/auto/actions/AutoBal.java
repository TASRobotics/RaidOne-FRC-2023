package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.submodules.Chassis;

import frc.robot.Constants.AutoConstants;

public class AutoBal implements Action{

    public double prevPitch = 0;

    public boolean isDone(){
        // TODO: Tune/fix this
        if (prevPitch < Chassis.getInstance().getPeriodicIO().pitch + 0.75
           && prevPitch > Chassis.getInstance().getPeriodicIO().pitch - 0.75) {
            return true;
        } else { return false; }
    }

    public void update(){

        double chassisSpeed = Chassis.getInstance().getPeriodicIO().pitch * AutoConstants.AUTOBAL_MULTIPLIER;
        prevPitch = Chassis.getInstance().getPeriodicIO().pitch;

        if (Chassis.getInstance().getPeriodicIO().pitch >= 3) {
             
            Chassis.getInstance().setPercentSpeed(chassisSpeed, chassisSpeed); 

        } else if (Chassis.getInstance().getPeriodicIO().pitch <= -3) {

            Chassis.getInstance().setPercentSpeed(chassisSpeed, chassisSpeed);

        }
        return;
    }

    public void initialize(){

    }

    public void done(){
        SmartDashboard.putNumber("Final pitch", Chassis.getInstance().getPeriodicIO().pitch);
    }
    
}
