package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.submodules.Chassis;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.wrappers.LazyCANSparkMax;
import java.util.Arrays;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

public class AutoBalPID implements Action{

    public double prevPitch = 0;
    public double initialPitch = 0;
    public boolean onPlatform = false;

    //pitch compensation multiplier is the scaling for pitch compensation when robot is on
    //the platform. increase to make robot scale faster.
    //if you want a smaller range but higher speed just add a constant and lower the multiplier
    public double pitchCompensationMultiplier = 1;

    // no speed modifier; if you want to make it faster, change the pid values
    public SparkMaxPIDController pidLeft;
    public SparkMaxPIDController pidRight;
    public double setVelocityRight;
    public double setVelocityLeft;

    public boolean isDone(){
        if (
         //   prevPitch < Chassis.getInstance().getPeriodicIO().pitch + 0.75
          // && prevPitch > Chassis.getInstance().getPeriodicIO().pitch - 0.75 &&
           //&& 
           onPlatform==true&& Chassis.getInstance().getPeriodicIO().pitch<initialPitch+2 
           && Chassis.getInstance().getPeriodicIO().pitch>initialPitch-1
           ) {
            System.out.println("isDone returned true");
            pidLeft.setReference(0, CANSparkMax.ControlType.kVelocity);
            pidRight.setReference(0, CANSparkMax.ControlType.kVelocity);
            return true;
        } else { return false; }
        //return false;
    }

    public void update(){
        //do important data collection stuff
        setVelocityRight = 0.75;//not sure where setpoint should be
        setVelocityLeft = 0.75;//maybe setpoint should be 
        prevPitch = Chassis.getInstance().getPeriodicIO().pitch;
        System.out.println("AutoBalPID Running...");
        System.out.println("Pitch: "+Chassis.getInstance().getPeriodicIO().pitch);
        System.out.println("I Accumulation: "+((pidRight.getIAccum()+pidLeft.getIAccum())/2));

        //if not on platform, move at setvelocity. if on platform, move at speed proportional to pitch
        if(onPlatform==false){
            pidLeft.setReference(setVelocityLeft*ChassisConstants.MPSToRPM, CANSparkMax.ControlType.kVelocity);
            pidRight.setReference(setVelocityRight*ChassisConstants.MPSToRPM, CANSparkMax.ControlType.kVelocity);
        } else {
            pidLeft.setReference(pitchCompensationMultiplier*Chassis.getInstance().getPeriodicIO().pitch
            *ChassisConstants.MPSToRPM, CANSparkMax.ControlType.kVelocity);
            pidRight.setReference(pitchCompensationMultiplier*Chassis.getInstance().getPeriodicIO().pitch
            *ChassisConstants.MPSToRPM, CANSparkMax.ControlType.kVelocity);
        }

        if(Chassis.getInstance().getPeriodicIO().pitch>=initialPitch+5){
            onPlatform = true;
            System.out.println("Climbing onto platform...");
        }


        return;
    }

    public void initialize(){
        onPlatform = false;
        initialPitch = Chassis.getInstance().getPeriodicIO().pitch;
        pitchCompensationMultiplier = 0.07;

        //initializing SparkMaxPIDControllers and PID constants
        //i commented it out, maybe want to make pidcontroller public?
        pidRight = Chassis.getInstance().getPIDController(false);
        pidLeft = Chassis.getInstance().getPIDController(true);

        pidRight.setFF(AutoConstants.AUTOBALPID_KFF);
        pidRight.setP(AutoConstants.AUTOBALPID_KP);
        pidRight.setI(AutoConstants.AUTOBALPID_KI);
        pidRight.setD(AutoConstants.AUTOBALPID_KD);
        pidRight.setIZone(AutoConstants.AUTOBALPID_KIZONE);
        pidRight.setOutputRange(AutoConstants.AUTOBALPID_KMINOUTPUT,
            AutoConstants.AUTOBALPID_KMAXOUTPUT);
        
        pidLeft.setFF(AutoConstants.AUTOBALPID_KFF);
        pidLeft.setP(AutoConstants.AUTOBALPID_KP);
        pidLeft.setI(AutoConstants.AUTOBALPID_KI);
        pidLeft.setD(AutoConstants.AUTOBALPID_KD);
        pidLeft.setIZone(AutoConstants.AUTOBALPID_KIZONE);
        pidLeft.setOutputRange(AutoConstants.AUTOBALPID_KMINOUTPUT,
            AutoConstants.AUTOBALPID_KMAXOUTPUT);
            



    }

    public void done(){
        SmartDashboard.putNumber("Final pitch", Chassis.getInstance().getPeriodicIO().pitch);
    }
    
}

