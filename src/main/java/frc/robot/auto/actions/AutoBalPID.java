package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.submodules.Chassis;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ChassisConstants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

public class AutoBalPID implements Action{

    public double prevPitch = 0;
    public double initialPitch = 0;
    public boolean onPlatform = false;

    // no speed modifier; if you want to make it faster, change the pid values
    public SparkMaxPIDController pidLeft;
    public SparkMaxPIDController pidRight;
    public double placeholderSetpointRight;
    public double placeholderSetpointLeft;

    public boolean isDone(){
        if (
         //   prevPitch < Chassis.getInstance().getPeriodicIO().pitch + 0.75
          // && prevPitch > Chassis.getInstance().getPeriodicIO().pitch - 0.75 &&
           //&& 
           onPlatform==true&& Chassis.getInstance().getPeriodicIO().pitch<initialPitch+2 
           && Chassis.getInstance().getPeriodicIO().pitch>initialPitch-1
           ) {
            System.out.println("isDone returned true");
            Chassis.getInstance().setPercentSpeed(0, 0);
            return true;
        } else { return false; }
        //return false;
    }

    public void update(){
        //do important data collection stuff
        placeholderSetpointRight = 10;//not sure where setpoint should be
        placeholderSetpointLeft = 10;//maybe setpoint should be 
        prevPitch = Chassis.getInstance().getPeriodicIO().pitch;
        System.out.println("AutoBalPID Running...");
        System.out.println("Pitch: "+Chassis.getInstance().getPeriodicIO().pitch);
        pidLeft.setReference(placeholderSetpointLeft*ChassisConstants.MPSToRPM, CANSparkMax.ControlType.kVelocity);
        pidRight.setReference(placeholderSetpointRight*ChassisConstants.MPSToRPM, CANSparkMax.ControlType.kVelocity);

        if(Chassis.getInstance().getPeriodicIO().pitch>=initialPitch+5){
            onPlatform = true;
            System.out.println("Climbing onto platform...");
        }


        return;
    }

    public void initialize(){
        onPlatform = false;
        initialPitch = Chassis.getInstance().getPeriodicIO().pitch;

        //initializing SparkMaxPIDControllers and PID constants
        //i commented it out, maybe want to make pidcontroller public?
        //pidRight = Chassis.getInstance().getRightLeader().getPIDController();
        //pidLeft = Chassis.getInstance().getLeftLeader().getPIDController();

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

