package frc.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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
    boolean hold = false;

    double weightSpeed = 0.0;
    double avgTriggerR = 0.0;
    double avgTriggerL = 0.0;

    public void onLoop() {
        double leftY = master.getLeftY() * val * 0.9;
        switchFront = master.getRightStickButton();
        if(switchFront && !prevSwitchFront) {
            val *= -1;
        }
        prevSwitchFront = switchFront;
        
        chassis.curvatureDrive(leftY, -master.getRightX() * 0.5, Math.abs(master.getLeftY()) < Constants.DEADBAND);

        int adjustForward = master.getYButton() ? 1 : 0;
        int adjustReverse = master.getAButton() ? 1 : 0;
        int adjustLeft = master.getXButton() ? 1 : 0;
        int adjustRight = master.getBButton() ? 1 : 0;
        if (master.getYButton()) {
            chassis.setPercentSpeed(-0.05 * adjustForward * driveMultiplier, -0.05 * adjustForward * driveMultiplier);
        } else if (master.getAButton()) {
            chassis.setPercentSpeed(0.05 * adjustReverse * driveMultiplier, 0.05 * adjustReverse * driveMultiplier);
        } else if (master.getXButton()) {
            chassis.setPercentSpeed(0.05 * adjustLeft * driveMultiplier, -0.05 * adjustLeft * driveMultiplier);
        } else if (master.getBButton()) {
            chassis.setPercentSpeed(-0.05 * adjustRight * driveMultiplier, 0.05 * adjustRight * driveMultiplier);
        }
        if(f&&partner.getXButton()){
            //chassis.teleopSmartHold();
            System.out.println("aaaa");
        }
        else if(partner.getXButtonPressed()){
            chassis.smartHold();
            System.out.println("eeee");
            f = true;
        }
        else{
            chassis.normal();
            f = false;
        if (master.getRightBumper() || partner.getRightBumper()) {
            driveMultiplier = 4.0;
        } else {
            driveMultiplier = 1.0;
        }

        avgTriggerR = ((master.getRightTriggerAxis() + partner.getRightTriggerAxis()) / 2);
        avgTriggerL = ((master.getLeftTriggerAxis() + partner.getLeftTriggerAxis()) / 2);

        // if (master.getRightBumperPressed() || partner.getRightBumperPressed()) {
            // if (weightShifter.getWeightPos() >= 13) {
                // weightShifter.setVelocity(-1);           
            // } else if (weightShifter.getWeightPos() <= 2) {
                // weightShifter.setVelocity(1);
            // }
        // }
// 
        // else if (master.getBButtonPressed() || partner.getBButtonPressed()) {
            // weightShifter.reset();
        // }
// 
        // if (master.getXButtonPressed()) { weightShifter.desiredVel = 0; }
        
        if ((avgTriggerR > avgTriggerL) && (avgTriggerR >= 0.1)) {
            weightSpeed = avgTriggerR;
        } else if ((avgTriggerR < avgTriggerL) && (avgTriggerL >= 0.1)) {
            weightSpeed = -avgTriggerL;
        } else if (master.getRightBumper()) {
            weightSpeed = 0.5;
        } else if (master.getLeftBumper()) {
            weightSpeed = -0.5;
        } else { weightSpeed = 0.0; }

        weightShifter.setVelocity(weightSpeed);

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
        }
        
    }

 