package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.submodules.SubmoduleManager;
import frc.robot.submodules.Chassis;
import frc.robot.auto.AutoRunner;
import frc.robot.teleop.Teleop;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

  private static final Chassis chassis = Chassis.getInstance();

  private AutoRunner autoRunner;
  private static final Teleop teleop = Teleop.getInstance();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // register all submodules here
    submoduleManager.setSubmodules(
      chassis
    );
    submoduleManager.onInit();

    autoRunner = new AutoRunner();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    submoduleManager.onStop(Timer.getFPGATimestamp());
    autoRunner.stop();
    System.out.println("dkfajsd");
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    submoduleManager.onStart(Timer.getFPGATimestamp());
    autoRunner.readSendableSequence();
    autoRunner.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double timestamp = Timer.getFPGATimestamp();
    submoduleManager.onLoop(timestamp);
    autoRunner.onLoop(timestamp);
  }

  @Override
  public void teleopInit() {
    // Stop the autonomous
    autoRunner.stop();

    teleop.onStart();
    submoduleManager.onStart(Timer.getFPGATimestamp());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    teleop.onLoop();
    double timestamp = Timer.getFPGATimestamp();
    submoduleManager.onLoop(timestamp);
  }

  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
