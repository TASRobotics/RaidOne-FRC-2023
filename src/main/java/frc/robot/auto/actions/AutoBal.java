package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.submodules.Chassis;
import java.util.Arrays;

import frc.robot.Constants.AutoConstants;

public class AutoBal implements Action{

    public double prevPitch = 0;
    public double initialPitch = 0;
    public int autoBalMode = 0;

    //speed modifier multiplies motor speeds by itself, where 1 is the slow base speed used for testing
    public double speedModifier = 1;
    
    /*
    autoBalMode 0 is when robot moves forward until it detects robot is tilting back
    autoBalMode 1 is when robot moves forward with compensation from pitch value, until pitch is close to 0
    autoBalMode 2 ends the code after robot reaches pitch of 0 (should end up on top of platform);

    */

    public AutoBal() {}

    public boolean isDone(){
        // TODO: Tune/fix this
        if (
           //prevPitch < Chassis.getInstance().getPeriodicIO().pitch + 0.75
            //&& prevPitch > Chassis.getInstance().getPeriodicIO().pitch - 0.75 &&
           //&& 
           autoBalMode == 1 && Chassis.getInstance().getPeriodicIO().pitch < initialPitch + 2 
                && Chassis.getInstance().getPeriodicIO().pitch > initialPitch - 1
           //autoBalMode==1&&Chassis.getInstance().getPeriodicIO().pitch<initialPitch+13&&Chassis.getInstance().getPeriodicIO().pitch>initialPitch+11
           ) {
            System.out.println("isDone returned true");
            Chassis.getInstance().setPercentSpeed(0, 0);
            return true;
        } else { return false; }
    }

    public void update(){
        System.out.println(autoBalMode);
        speedModifier = 0.8;
        //NOTE: i have changed autobal multiplier to 0.01 for testing purposes so robot is much slower than usual
        double chassisSpeed = Chassis.getInstance().getPeriodicIO().pitch * AutoConstants.AUTOBAL_MULTIPLIER * speedModifier;
        prevPitch = Chassis.getInstance().getPeriodicIO().pitch;
        System.out.println("AutoBal Running...");
        System.out.println("Pitch: "+ Chassis.getInstance().getPeriodicIO().pitch);
        if(autoBalMode==0){
            Chassis.getInstance().setPercentSpeed(0.3*speedModifier, 0.3*speedModifier);
            //SmartDashboard.putNumber("ini+5", initialPitch+5);
            //SmartDashboard.putNumber("pitch read", Chassis.getInstance().getPeriodicIO().pitch);
            if(Chassis.getInstance().getPeriodicIO().pitch >= initialPitch + 5){
                autoBalMode = 1;
                Chassis.getInstance().setPercentSpeed(0, 0);
            }
        }
        if(autoBalMode == 1){
            if (Chassis.getInstance().getPeriodicIO().pitch >= 3) {
                System.out.println("Head(battery side) tilting down");
                Chassis.getInstance().setPercentSpeed(chassisSpeed, chassisSpeed); 

            } else if (Chassis.getInstance().getPeriodicIO().pitch <= -3) {
                System.out.println("Head(battery side) tilting up");
                Chassis.getInstance().setPercentSpeed(chassisSpeed, chassisSpeed);

            }
        }
        return;
    }

    public void initialize(){
        autoBalMode = 0;
        initialPitch = Chassis.getInstance().getPeriodicIO().pitch;
    }

    public void done(){
        SmartDashboard.putNumber("Final pitch", Chassis.getInstance().getPeriodicIO().pitch);
    }
    
}
