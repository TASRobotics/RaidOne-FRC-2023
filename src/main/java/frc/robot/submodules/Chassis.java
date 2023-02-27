package frc.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//import com.ctre.phoenix.music;

import frc.robot.wrappers.InactiveCompressor;
import frc.robot.wrappers.LazyVictorSPX;
import frc.robot.wrappers.LazyTalonSRX;
import frc.robot.wrappers.LogicalTalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ChassisConstants;
import frc.robot.pathing.TrajectoryFollower;
import frc.robot.pathing.VelocityController;
import frc.robot.utils.JoystickUtils;
import frc.robot.Constants;

/**
 * attempt to change chassis from talon to sparkmax!
 * https://docs.revrobotics.com/sparkmax/software-resources/migrating-ctre-to-rev
 */

public class Chassis extends Submodule {
    public static class PeriodicIO {
        // Inputs
        public DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);

        public double leftPosition = 0; // in meters
        public double rightPosition = 0; // in meters
        public Rotation2d heading = new Rotation2d(0);

        public double actualLeftVelocity = 0; // in m/s
        public double actualRightVelocity = 0; // in m/s

        public double x = 0;
        public double y = 0;
        public Rotation2d rotation = new Rotation2d(0);

        public double pitch = 0;

        // Outputs
        public double leftPercent = 0.0;
        public double rightPercent = 0.0;

        public double desiredLeftVelocity = 0.0;
        public double desiredRightVelocity = 0.0;

        public double leftFF = 0.0;
        public double rightFF = 0.0;
    }

    /** Enum controlling gear shift */
    public static enum GearShift {
        HIGH_TORQUE, LOW_TORQUE, OFF
    }

    /** Enum controlling control state */
    public static enum ControlState {
        OPEN_LOOP, PATH_FOLLOWING, AUTOBALANCE
    }

    /** Motors */
    public final LazyTalonSRX mLeftLeader = new LazyTalonSRX(ChassisConstants.LEFT_LEADER_ID);
    private final LazyVictorSPX mLeftFollowerA = new LazyVictorSPX(ChassisConstants.LEFT_FOLLOWER_A_ID);
    private final LazyVictorSPX mLeftFollowerB = new LazyVictorSPX(ChassisConstants.LEFT_FOLLOWER_B_ID);

    public final LazyTalonSRX mRightLeader = new LazyTalonSRX(ChassisConstants.RIGHT_LEADER_ID);
    private final LazyVictorSPX mRightFollowerA = new LazyVictorSPX(ChassisConstants.RIGHT_FOLLOWER_A_ID);    
    private final LazyVictorSPX mRightFollowerB = new LazyVictorSPX(ChassisConstants.RIGHT_FOLLOWER_B_ID);    

    /** Sensors */
    private final PigeonIMU mImu = new PigeonIMU(ChassisConstants.IMU_ID);
    private RelativeEncoder encoderL;
    private RelativeEncoder encoderR;

    /** Controllers */
    private DifferentialDriveOdometry mOdometry;
    private TrajectoryFollower trajectoryFollower;
    // private VelocityController leftVelController, rightVelController;
    private VelocityController velocityController;
    private double leftPrevVel, rightPrevVel;
    private SparkMaxPIDController mPIDControllerR;
    private SparkMaxPIDController mPIDControllerL;

    private ControlState controlState = ControlState.OPEN_LOOP;
    private PeriodicIO periodicIO = new PeriodicIO();
   

    private Chassis() {}
    private static Chassis instance = null;
    public static Chassis getInstance() {
        if(instance == null) {
            instance = new Chassis();
        }
        return instance;
    }

    @Override
    public void onInit() {
        /** Config factory default for all motors */
        mLeftLeader.configFactoryDefault();
        mLeftFollowerA.configFactoryDefault();
        mLeftFollowerB.configFactoryDefault();
        mRightLeader.configFactoryDefault();
        mRightFollowerA.configFactoryDefault();
        mRightFollowerB.configFactoryDefault();

        /** Config factory default for sensors */
        mImu.configFactoryDefault();

        /** Config followers */
        mLeftFollowerA.follow(mLeftLeader);
        mLeftFollowerB.follow(mLeftLeader);
        mRightFollowerA.follow(mRightLeader);
        mRightFollowerB.follow(mRightLeader);

        /** Inverts motors */
        mLeftLeader.setInverted(false);
        mLeftFollowerA.setInverted(false);
        mLeftFollowerB.setInverted(false);
        mRightLeader.setInverted(true);
        mRightFollowerA.setInverted(true);
        mRightFollowerB.setInverted(true);

        /** inverts encoder*/
        //encoderL = mLeftLeader.getEncoder();
        //encoderL.setInverted(true);
        //encoderR = mRightLeader.getEncoder();
        //encoderR.setInverted(false);

        mLeftLeader.enableVoltageCompensation(true);//Constants.VOLTAGE_COMPENSATION);
        mRightLeader.enableVoltageCompensation(true);//Constants.VOLTAGE_COMPENSATION);

        /** Config ramp rate */
        //mLeftLeader.setOpenLoopRampRate(ChassisConstants.RAMP_RATE);
        //mLeftLeader.setClosedLoopRampRate(0);
        //mRightLeader.setOpenLoopRampRate(ChassisConstants.RAMP_RATE);
        //mRightLeader.setClosedLoopRampRate(0);

        /**
         * would someone kindly explain what this is ^^
         */
        /*
        mLeftLeader.configContinuousCurrentLimit(30, Constants.TIMEOUT_MS);
        mLeftLeader.configPeakCurrentLimit(80, Constants.TIMEOUT_MS);
        mLeftLeader.configPeakCurrentDuration(1000, Constants.TIMEOUT_MS);
        mRightLeader.configContinuousCurrentLimit(30, Constants.TIMEOUT_MS);
        mRightLeader.configPeakCurrentLimit(80, Constants.TIMEOUT_MS);
        mRightLeader.configPeakCurrentDuration(1000, Constants.TIMEOUT_MS);
        */
        //mLeftLeader.setSmartCurrentLimit(45);
        //mRightLeader.setSmartCurrentLimit(45);

        /** Config Talon PID */
        /* 
        mPIDControllerR = mRightLeader.getPIDController();
        mPIDControllerL = mLeftLeader.getPIDController();
        mPIDControllerR.setP(ChassisConstants.kP);
        mPIDControllerL.setP(ChassisConstants.kP);
        mPIDControllerL.setIZone(ChassisConstants.PID_LOOP_IDX);
        mPIDControllerR.setIZone(ChassisConstants.PID_LOOP_IDX);
        //mPIDControllerL.setFF(0.000156);
        //mPIDControllerR.setFF(0.000156);
        mPIDControllerL.setOutputRange(-1,1);
        mPIDControllerR.setOutputRange(-1,1);
        */


        /** Config after imu init */
        trajectoryFollower = new TrajectoryFollower(ChassisConstants.DRIVE_KINEMATICS);
        // leftVelController = new VelocityController(ChassisConstants.LEFT_kV, ChassisConstants.LEFT_kA, ChassisConstants.LEFT_kP);
        // rightVelController = new VelocityController(ChassisConstants.RIGHT_kV, ChassisConstants.RIGHT_kA, ChassisConstants.RIGHT_kP);
        velocityController = new VelocityController(ChassisConstants.kS, ChassisConstants.kV, ChassisConstants.kA, ChassisConstants.kP);
        leftPrevVel = 0.0;
        rightPrevVel = 0.0;

        // Reset sensors (must happen before odom init)
        zero();

        mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);

        setBrakeMode(true);

        /** Camera */
        // UsbCamera cam1 =  CameraServer.startAutomaticCapture(0);
        // cam1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);


        // Logger.configureLoggingAndConfig(this, false);
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;

        periodicIO = new PeriodicIO();

        stop();
        zero();
        resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

        setBrakeMode(true);
    }

    @Override
    public void run() {
        switch(controlState) {
            case OPEN_LOOP:
                mLeftLeader.set(ControlMode.PercentOutput, periodicIO.leftPercent);
                mRightLeader.set(ControlMode.PercentOutput, periodicIO.rightPercent);
                break;  

            case PATH_FOLLOWING:
                //mPIDControllerL.setFF((periodicIO.leftFF));
                //mPIDControllerR.setFF((periodicIO.rightFF));
                //System.out.println("ioff: " + (float)(periodicIO.leftFF));
                //System.out.println("ioff: " + (float)(periodicIO.rightFF));
                //mPIDControllerL.setFF(ChassisConstants.kV);
                //mPIDControllerR.setFF(ChassisConstants.kV);
                mPIDControllerL.setFF(1.0/6000.0); 
                mPIDControllerR.setFF(1.0/6000.0);
                mPIDControllerL.setReference(periodicIO.desiredLeftVelocity*ChassisConstants.MPSToRPM, ControlType.kVelocity);
                mPIDControllerR.setReference(periodicIO.desiredRightVelocity*ChassisConstants.MPSToRPM, ControlType.kVelocity);
                SmartDashboard.putNumber("left input RPM", periodicIO.desiredLeftVelocity*ChassisConstants.MPSToRPM);
                SmartDashboard.putNumber("right input RPM", periodicIO.desiredRightVelocity*ChassisConstants.MPSToRPM);
                /* 
                SmartDashboard.putNumber("left applied output", mLeftLeader.getAppliedOutput());
                SmartDashboard.putNumber("left ouput current", mLeftLeader.getOutputCurrent());
                SmartDashboard.putNumber("right applied output", mRightLeader.getAppliedOutput());
                SmartDashboard.putNumber("right ouput current", mRightLeader.getOutputCurrent());
                SmartDashboard.putNumber("desired left vel m/s" , periodicIO.desiredLeftVelocity);
                SmartDashboard.putNumber("desired right vel m/s" , periodicIO.desiredRightVelocity);
                */
                break;
        }
    }

    @Override
    public void update(double timestamp) {
        //SmartDashboard.putNumber("encoderL pos", encoderL.getPosition());
        //SmartDashboard.putNumber("encoderR pos", encoderR.getPosition());

        // Autobalance
        periodicIO.pitch = mImu.getPitch();
        
        //periodicIO.leftPosition = encoderL.getPosition() * ChassisConstants.kEncoderDistancePerRevolution;
        //periodicIO.rightPosition = encoderR.getPosition() * ChassisConstants.kEncoderDistancePerRevolution;

        // velocity
        //periodicIO.actualLeftVelocity = encoderL.getVelocity() * ChassisConstants.kEncoderDistancePerRevolution/60;
        //periodicIO.actualRightVelocity = encoderR.getVelocity() * ChassisConstants.kEncoderDistancePerRevolution/60;

        periodicIO.heading = Rotation2d.fromDegrees(rescale180(mImu.getYaw()));

        Pose2d updatedPose = updateOdometry();//trajectoryFollower.s().poseMeters;//updateOdometry();
        periodicIO.x = updatedPose.getX();
        periodicIO.y = updatedPose.getY();
        periodicIO.rotation = updatedPose.getRotation();
        SmartDashboard.putNumber("x",periodicIO.x);
        SmartDashboard.putNumber("y",periodicIO.y);
        SmartDashboard.putNumber("heading deg",periodicIO.heading.getDegrees());
        SmartDashboard.putNumber("actual left vel m/s", periodicIO.actualLeftVelocity);
        SmartDashboard.putNumber("actual right vel m/s", periodicIO.actualRightVelocity);
        SmartDashboard.putNumber("heading", periodicIO.heading.getDegrees());
        SmartDashboard.putNumber("pitch", mImu.getPitch());

        //SmartDashboard.putNumber("left enc vel", encoderL.getVelocity());
        //SmartDashboard.putNumber("Right enc vel", encoderR.getVelocity());
        


        if(controlState == ControlState.PATH_FOLLOWING) {
            /** WHY DO I NEED TO MAKE THIS NEGATIVE!?! */
            double leftVel = -trajectoryFollower.update(updatedPose).leftMetersPerSecond;
            double rightVel = -trajectoryFollower.update(updatedPose).rightMetersPerSecond;
            // double rightVel = trajectoryFollower.update(updatedPose).leftMetersPerSecond;
            // double leftVel = trajectoryFollower.update(updatedPose).rightMetersPerSecond;

            SmartDashboard.putNumber("trajectory left", leftVel);
            SmartDashboard.putNumber("trajectory right", rightVel);

            /** Calculate accel */
            double leftAccel = leftVel - leftPrevVel;
            double rightAccel = rightVel - rightPrevVel;
            leftPrevVel = leftVel;
            rightPrevVel = rightVel;

            // periodicIO.leftFF = leftVelController.updateFF(leftVel, leftAccel);
            // periodicIO.rightFF = rightVelController.updateFF(rightVel, rightAccel);
            periodicIO.leftFF = velocityController.updateFF(leftVel, leftAccel);
            periodicIO.rightFF = velocityController.updateFF(rightVel, rightAccel);

            //setVelocity(leftVel, rightVel); //WHY DOESN THIS WORK?? it was used in last yrs code???
            setVelocity(periodicIO.leftFF, periodicIO.rightFF);
        }
    }

    /** Stops the compressor and all chassis motors */
    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;

        periodicIO.leftPercent = 0.0;
        periodicIO.rightPercent = 0.0;

        mLeftLeader.set(ControlMode.PercentOutput, 0.0);
        mRightLeader.set(ControlMode.PercentOutput, 0.0);
        //prob need to change brakemode to coast!
        setBrakeMode(false);
    }

    /**
     * Sets percent speed [-1, 1]
     * 
     * @param left left speed
     * @param right right speed
     */
    public void setPercentSpeed(double left, double right) {
        periodicIO.leftPercent = left;
        periodicIO.rightPercent = right;
    }

    /**
     * Sets velocity speed [-1, 1]
     * 
     * @param left left speed
     * @param right right speed
     */
    public void setVelocity(double left, double right) {
        periodicIO.desiredLeftVelocity = left;
        periodicIO.desiredRightVelocity = right;
    }


    /**
     * A better arcade drive
     * 
     * @param throttle usually the left y axis of a controller
     * @param turn usually the right x axis of a controllers
     * @param quickTurn basically an arcade drive switch
     */
    public void curvatureDrive(double throttle, double turn, boolean quickTurn) {
        /** Set deadband to all inputs */
        // throttle = JoystickUtils.deadband(JoystickUtils.monomialScale(throttle, ChassisConstants.MONOMIAL_SCALE, 1));
        // turn = JoystickUtils.deadband(JoystickUtils.monomialScale(turn, ChassisConstants.MONOMIAL_SCALE, 1));

        throttle = JoystickUtils.deadband(throttle);
        turn = JoystickUtils.deadband(turn);

        // Compute velocity, right stick = curvature if no quickturn, else power
        double leftSpeed = throttle + (quickTurn ? turn : Math.abs(throttle) * turn);
        double rightSpeed = throttle - (quickTurn ? turn : Math.abs(throttle) * turn);
    
        // Normalize velocity
        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
            leftSpeed /= maxMagnitude;
            rightSpeed /= maxMagnitude;
        }
        periodicIO.leftPercent = leftSpeed;
        periodicIO.rightPercent = rightSpeed;
    }

    /**
     * Aracde drive
     * 
     * @param throttle forward
     * @param turn turn
     */
    public void arcadeDrive(double throttle, double turn) {
        throttle = JoystickUtils.deadband(JoystickUtils.monomialScale(throttle, ChassisConstants.MONOMIAL_SCALE, 1));
        turn = JoystickUtils.deadband(JoystickUtils.monomialScale(turn, ChassisConstants.MONOMIAL_SCALE, 1));
        periodicIO.leftPercent = throttle + turn;
        periodicIO.rightPercent = throttle - turn;
    }

    /**
     * Tank drive
     * 
     * @param left percent speed
     * @param right percent speed
     */
    public void tankDrive(double left, double right) {
        // left = JoystickUtils.deadband(JoystickUtils.monomialScale(left, ChassisConstants.MONOMIAL_SCALE, 1));
        // right = JoystickUtils.deadband(JoystickUtils.monomialScale(right, ChassisConstants.MONOMIAL_SCALE, 1));
        left = JoystickUtils.deadband(left);
        right = JoystickUtils.deadband(right);
        periodicIO.leftPercent = left;
        periodicIO.rightPercent = right;
    }

    /** Zeros all sensors */
    @Override
    public void zero() {
        resetEncoders();
        zeroHeading();
        //zeroPitch();
    }

    /** Resets drive encoders to 0 */
    public void resetEncoders() {
        //encoderL.setPosition(0);
        //encoderR.setPosition(0);
    }

    /** Zeros IMU heading */
    public void zeroHeading() {
        mImu.setYaw(0);
    }

    public void zeroPitch(){
        //mImu.setPitch(0);
    }

    /**
     * Returns Periodic IO
     * 
     * @return current periodic io
     */
    public PeriodicIO getPeriodicIO() {
        return periodicIO;
    }

    /**
     * Rescales an angle to [-180, 180]
     * 
     * @param angle the angle to be rescalled
     * @return rescalled angle
     */
    private double rescale180(double angle) {
        return angle - 360.0 * Math.floor((angle + 180.0) * (1.0 / 360.0));
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        mOdometry.resetPosition(periodicIO.rotation, 0, 0, pose);
    }

    /** Updates odom */
    private Pose2d updateOdometry() {
        return mOdometry.update(periodicIO.heading, periodicIO.leftPosition, periodicIO.rightPosition);
    }

    /**
     * Sets the brake mode to brake or coast.
     * 
     * @param brake whether to brake or not
     */
    public void setBrakeMode(boolean brake) {
        /* 
        if (brake) {
            mRightLeader.setIdleMode(IdleMode.kBrake);
            mLeftLeader.setIdleMode(IdleMode.kBrake);
            mLeftFollowerA.setIdleMode(IdleMode.kBrake);
            mLeftFollowerB.setIdleMode(IdleMode.kBrake);
            mRightFollowerA.setIdleMode(IdleMode.kBrake);
            mRightFollowerB.setIdleMode(IdleMode.kBrake);
        } else {
            mRightLeader.setIdleMode(IdleMode.kCoast);
            mLeftLeader.setIdleMode(IdleMode.kCoast);
            mLeftFollowerA.setIdleMode(IdleMode.kCoast);
            mLeftFollowerB.setIdleMode(IdleMode.kCoast);
            mRightFollowerA.setIdleMode(IdleMode.kCoast);
            mRightFollowerB.setIdleMode(IdleMode.kCoast);
        }
        */
    }

    /**
     * Makes the drive start following a Path.
     * 
     * @param path           the path to follow
     * @param zeroAllSensors whether to zero all sensors to the first point
     */
    public void setDrivePath(Trajectory trajectory) {
        if (trajectoryFollower != null) {
            // Stops the drive
            stop();

            // Reset & start trajectory follower
            trajectoryFollower.reset();
            trajectoryFollower.start(trajectory);

            controlState = ControlState.PATH_FOLLOWING;
        }
    }

    /**
     * Returns whether the drive has finished following a path.
     * 
     * @return if the drive is finished pathing
     */
    public boolean isFinishedWithPath() {
        if (trajectoryFollower == null || controlState != ControlState.PATH_FOLLOWING) {
            return false;
        }
        return trajectoryFollower.isFinished();
    }

    public SparkMaxPIDController getPIDController(boolean isLeft) {
        if (isLeft){
            return mPIDControllerL;
        } else {
            return mPIDControllerR;
        }
    }
}
