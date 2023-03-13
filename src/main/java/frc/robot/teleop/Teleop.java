package frc.robot.teleop;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.WeightConstants;
import frc.robot.submodules.Chassis;
import frc.robot.submodules.WeightShifter;

public class Teleop {

    private static Teleop instance = null;

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    private Teleop() {
    }

    private XboxController master = new XboxController(0);
    private XboxController partner = new XboxController(1);

    private static Chassis chassis = Chassis.getInstance();
    private static WeightShifter weightShifter = WeightShifter.getInstance();

    /**
     * Runs at the start of teleop.
     */
    public void onStart() {

        chassis.zero();
        chassis.setBrakeMode(false);
    }

    /**
     * Continuously loops in teleop.
     */
    boolean shiftState = false, prevShiftState = false, driveState = false, prevDriveState = false, switchFront = false, prevSwitchFront = false;

    // int drive = 0;
    boolean shift = false;
    int val = 1;
    // String message = "";

    boolean prevWeightState = false;

    public void onLoop() {
        double leftY = master.getLeftY() * val * 0.9;
        switchFront = master.getRightStickButton();
        if(switchFront && !prevSwitchFront) {
            val *= -1;
        }
        prevSwitchFront = switchFront;
        
        chassis.curvatureDrive(leftY, -master.getRightX() * 0.3, Math.abs(master.getLeftY()) < Constants.DEADBAND);
/*
        if (master.getAButtonPressed() || partner.getAButtonPressed()) {
            weightShifter.punch();
        }

        if (master.getBButtonPressed() || partner.getBButtonPressed()) {
            weightShifter.reset();
        }

        if (master.getRightBumper() || partner.getRightBumper()) {
            weightShifter.setVelocity(0.5);
        } else if (master.getLeftBumper() || partner.getRightBumper()) {
            weightShifter.setVelocity(-0.5);
        }

        avgTriggerR = ((master.getRightTriggerAxis() + partner.getRightTriggerAxis()) / 2);
        avgTriggerL = ((master.getLeftTriggerAxis() + partner.getLeftTriggerAxis()) / 2);
        
        if (avgTriggerR > avgTriggerL) {
            weightSpeed = avgTriggerR;
        } else if (avgTriggerR < avgTriggerL) {
            weightSpeed = -avgTriggerL;
        } else { weightSpeed = 0; }

        weightShifter.setVelocity(weightSpeed); */

        // chassis.tankDrive(master.getLeftY(), master.getRightY());

        // driveState = master.getAButton();
        // if(driveState && !prevDriveState) {
        //     drive++;
        // }
        // switch(drive %= 3) {
        //     case 0: 
        //         chassis.curvatureDrive(-master.getRawAxis(1), master.getRawAxis(4), Math.abs(master.getRawAxis(1)) < Constants.DEADBAND);
        //         message = "Curvature Drive";
        //         break;
        //     case 1:
        //         chassis.tankDrive(-master.getRawAxis(1), -master.getRawAxis(5));
        //         message = "Tank Drive";
        //         break;
        //     case 2:
        //         chassis.arcadeDrive(-master.getRawAxis(1), master.getRawAxis(4));
        //         message = "Arcade Drive";
        //         break;
        // }
        // prevDriveState = driveState;
        // SmartDashboard.putString("Drive type", message);

        SmartDashboard.putNumber("x pose", chassis.getPeriodicIO().x);
        SmartDashboard.putNumber("y pose", chassis.getPeriodicIO().y);
        SmartDashboard.putNumber("rotation", chassis.getPeriodicIO().rotation.getDegrees());

        if(master.getStartButton()) {
            //climb.setState(EZClimbState.UP);
        } else {
            //climb.setState(EZClimbState.DOWN);
        }

        /** Shift */
        if(master.getRightBumper()) {
            //climb.setSpeed(master.getLeftTriggerAxis());
        } else {
            //intake.autoSet(master.getRightTriggerAxis() - master.getLeftTriggerAxis());
        }
        //intake.setState(partner.getLeftBumper() || partner.getRightBumper() ? IntakeState.DOWN : IntakeState.UP);
        //intake.setPercentSpeed(partner.getRightTriggerAxis() - partner.getLeftTriggerAxis());

        // Falcon punch
        boolean weightState = master.getYButton();

        if(master.getYButtonPressed()) {
            System.out.println("y pressed");
            if (Chassis.getInstance().getWeightPos() >= 26){
                Chassis.getInstance().setWeightPos(true);
                //System.out.println("weightshifter moved to 1");
                
            } else {
                Chassis.getInstance().setWeightPos(false);
                //System.out.println("weightshifter moved to 27");;
            }
            prevWeightState = weightState;
            }
            }
        
    }

 