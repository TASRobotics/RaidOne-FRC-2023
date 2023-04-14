package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.submodules.Chassis;

import frc.robot.Constants.AutoConstants;

public class AutoBal implements Action{

    public double prevPitch = 0;
    public double initialPitch = 0;
    public int autoBalMode = 0;
    private boolean reverse = false;
    private boolean stop = false;

    //speed modifier multiplies motor speeds by itself, where 1 is the slow base speed used for testing
    public double speedModifier = 5;
    
    /*
    autoBalMode 0 is when robot moves forward until it detects robot is tilting back
    autoBalMode 1 is when robot moves forward with compensation from pitch value, until pitch is close to 0
    autoBalMode 2 ends the code after robot reaches pitch of 0 (should end up on top of platform);

    */

    public AutoBal(boolean rev) {
        reverse = rev;
    }

    public AutoBal(){
        reverse = false;
    }

    public boolean isDone(){
        /*
        if (autoBalMode == 1 && Chassis.getInstance().getPeriodicIO().pitch < initialPitch + 2 
                && Chassis.getInstance().getPeriodicIO().pitch > initialPitch - 1
           ) {
            System.out.println("isDone returned true");
            Chassis.getInstance().setPercentSpeed(0, 0);
            return true;
        } else { return false; }
*/
        if(stop) return true;
        return false;
    }

    public void update(){
        //System.out.println(autoBalMode);
        speedModifier = 6000;
        if(reverse==false){
        //NOTE: i have changed autobal multiplier to 0.01 for testing purposes so robot is much slower than usual
        double chassisSpeed = Chassis.getInstance().getPeriodicIO().pitch * AutoConstants.AUTOBAL_MULTIPLIER * speedModifier;
        prevPitch = Chassis.getInstance().getPeriodicIO().pitch;
        //System.out.println("AutoBal Running...");
        //System.out.println("Pitch: "+ Chassis.getInstance().getPeriodicIO().pitch);
        //System.out.println("initial pitch: " + initialPitch);
        if(autoBalMode==0){
            Chassis.getInstance().setVelocity(-0.3*speedModifier, -0.3*speedModifier);
            //SmartDashboard.putNumber("ini+5", initialPitch+5);
            //SmartDashboard.putNumber("pitch read", Chassis.getInstance().getPeriodicIO().pitch);
            if(Chassis.getInstance().getPeriodicIO().pitch >= initialPitch + 7){ //5
                autoBalMode = 1;
                Chassis.getInstance().setVelocity(0, 0);
            }
        }
        if(autoBalMode == 1){
            if (Chassis.getInstance().getPeriodicIO().pitch >= initialPitch+1.5) {
                //System.out.println("Head(battery side) tilting down");
                Chassis.getInstance().setVelocity(-chassisSpeed, -chassisSpeed); 

            } else if (Chassis.getInstance().getPeriodicIO().pitch <= -3) {
                //System.out.println("Head(battery side) tilting up");
                Chassis.getInstance().setVelocity(-chassisSpeed, -chassisSpeed);

            }
            else{
                Chassis.getInstance().setVelocity(0,0);
                stop = true;
                done();
            }
        }}
        else{
            double chassisSpeed = 5*(-Chassis.getInstance().getPeriodicIO().pitch) * AutoConstants.AUTOBAL_MULTIPLIER * speedModifier;
        prevPitch = -Chassis.getInstance().getPeriodicIO().pitch;
        //System.out.println("AutoBal Running...");
        //System.out.println("Pitch: "+ Chassis.getInstance().getPeriodicIO().pitch);
        //System.out.println("initial pitch: " + initialPitch);
        if(autoBalMode==0){
            Chassis.getInstance().setVelocity(0.3*speedModifier, 0.3*speedModifier);            
            SmartDashboard.putNumber("ini", initialPitch);
            //SmartDashboard.putNumber("pitch read", Chassis.getInstance().getPeriodicIO().pitch);
            if(-Chassis.getInstance().getPeriodicIO().pitch >= initialPitch + 20){ //5
                autoBalMode = 1;
                Chassis.getInstance().setVelocity(0, 0);
                System.out.println("change auto bal");
            }
        }
        if(autoBalMode == 1){
            SmartDashboard.putNumber("cs: ", -chassisSpeed);
            /*
            if (-Chassis.getInstance().getPeriodicIO().pitch >= initialPitch+1.5) {
                //System.out.println("Head(battery side) tilting down");
                Chassis.getInstance().setPercentSpeed(-chassisSpeed, -chassisSpeed); 

            } else if (-Chassis.getInstance().getPeriodicIO().pitch <= -3) {
                //System.out.println("Head(battery side) tilting up");
                Chassis.getInstance().setPercentSpeed(-chassisSpeed, -chassisSpeed);


            }
            else{
                System.out.println("aaaaaaaaaaaaaa");
                Chassis.getInstance().setPercentSpeed(0,0);
                done();
            }*/
            Chassis.getInstance().setVelocity(-chassisSpeed, -chassisSpeed); 
        }
        }
        return;
    }

    public void initialize(){
        autoBalMode = 0;
        Chassis.getInstance().autobal();
        if(reverse==false)initialPitch = Chassis.getInstance().getPeriodicIO().pitch;
        else initialPitch = -Chassis.getInstance().getPeriodicIO().pitch;
    }

    public void done(){
        SmartDashboard.putNumber("Final pitch", Chassis.getInstance().getPeriodicIO().pitch);
    }
    
}
