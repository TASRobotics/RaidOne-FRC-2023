package frc.robot.submodules;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//import com.ctre.phoenix.music;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;

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

    public static enum GearShift {
        HIGH_TORQUE, LOW_TORQUE, OFF
    }

    /** Enum controlling control state */
    public static enum ControlState {
        OPEN_LOOP, PATH_FOLLOWING, AUTOBALANCE
    }

    /** Motors */
    private final CANSparkMax mLeftLeader = new CANSparkMax(ChassisConstants.LEFT_LEADER_ID, MotorType.kBrushless);
    private final CANSparkMax mLeftFollowerA = new CANSparkMax(ChassisConstants.LEFT_FOLLOWER_A_ID, MotorType.kBrushless);
    private final CANSparkMax mLeftFollowerB = new CANSparkMax(ChassisConstants.LEFT_FOLLOWER_B_ID, MotorType.kBrushless);

    private final CANSparkMax mRightLeader = new CANSparkMax(ChassisConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
    private final CANSparkMax mRightFollowerA = new CANSparkMax(ChassisConstants.RIGHT_FOLLOWER_A_ID, MotorType.kBrushless);    
    private final CANSparkMax mRightFollowerB = new CANSparkMax(ChassisConstants.RIGHT_FOLLOWER_B_ID, MotorType.kBrushless);

    //private final CANSparkMax mWeightShifter = new CANSparkMax(ChassisConstants.WEIGHTSHIFTER_ID, MotorType.kBrushless);
    //private final SparkMaxPIDController mWeightPID = mWeightShifter.getPIDController();
    //private final RelativeEncoder  mWeightEncoder = mWeightShifter.getEncoder();

    /** Sensors */
    private final PigeonIMU mImu = new PigeonIMU(ChassisConstants.IMU_ID);
    //private RelativeEncoder encoderL;
    //private RelativeEncoder encoderR;
    private final CANCoder encoderL = new CANCoder(1);
    private final CANCoder encoderR = new CANCoder(2);

    /** Controllers */
    private DifferentialDriveOdometry mOdometry;
    private TrajectoryFollower trajectoryFollower;
    // private VelocityController leftVelController, rightVelController;
    private VelocityController velocityController;
    private double leftPrevVel, rightPrevVel;
    private SparkMaxPIDController mPIDControllerR;
    private SparkMaxPIDController mPIDControllerL;

    /** Teleop acceleration limit */
    private SlewRateLimiter driveFilter = new SlewRateLimiter(ChassisConstants.SLEW_FILTER);
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
        mLeftLeader.restoreFactoryDefaults();
        mLeftFollowerA.restoreFactoryDefaults();
        mLeftFollowerB.restoreFactoryDefaults();

        mRightLeader.restoreFactoryDefaults();
        mRightFollowerA.restoreFactoryDefaults();
        mRightFollowerB.restoreFactoryDefaults();

        //mWeightShifter.restoreFactoryDefaults();

        /** Config factory default for sensors */
        mImu.configFactoryDefault();

        /** Config followers */
        mLeftFollowerA.follow(mLeftLeader);
        mLeftFollowerB.follow(mLeftLeader);

        mRightFollowerA.follow(mRightLeader);
        mRightFollowerB.follow(mRightLeader);

        /** Inverts motors */
        mLeftLeader.setInverted(true);        
        mRightLeader.setInverted(false);

        /** Config encoder */
        //encoderL = mLeftLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
        //encoderR = mRightLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
        //encoderL.setInverted(true);
        //encoderR.setInverted(false);
        encoderL.configSensorDirection(false);
        encoderR.configSensorDirection(true);
        encoderL.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        encoderR.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        mLeftLeader.enableVoltageCompensation(Constants.VOLTAGE_COMPENSATION);
        mRightLeader.enableVoltageCompensation(Constants.VOLTAGE_COMPENSATION);
        

        /** Config ramp rate */
        mLeftLeader.setOpenLoopRampRate(ChassisConstants.RAMP_RATE);
        mLeftLeader.setClosedLoopRampRate(0);
        mRightLeader.setOpenLoopRampRate(ChassisConstants.RAMP_RATE);
        mRightLeader.setClosedLoopRampRate(0);

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
        mLeftLeader.setSmartCurrentLimit(35);
        mRightLeader.setSmartCurrentLimit(35);
        mLeftFollowerA.setSmartCurrentLimit(35);
        mRightFollowerA.setSmartCurrentLimit(35);
        mLeftFollowerB.setSmartCurrentLimit(35);
        mRightFollowerB.setSmartCurrentLimit(35);

        /** Config Talon PID */
        mPIDControllerR = mRightLeader.getPIDController();
        mPIDControllerL = mLeftLeader.getPIDController();
        mPIDControllerR.setP(ChassisConstants.kP);
        mPIDControllerL.setP(ChassisConstants.kP);
        mPIDControllerL.setIZone(ChassisConstants.PID_LOOP_IDX);
        mPIDControllerR.setIZone(ChassisConstants.PID_LOOP_IDX);
        mPIDControllerL.setFF(1.0/6000.0); 
        mPIDControllerR.setFF(1.0/6000.0);
        //mPIDControllerL.setFF(0.000156);
        //mPIDControllerR.setFF(0.000156);
        mPIDControllerL.setOutputRange(-1,1);
        mPIDControllerR.setOutputRange(-1,1);

        /** Config weight shifter motor */
        //mWeightShifter.setIdleMode(IdleMode.kBrake);
        //mWeightShifter.setSmartCurrentLimit(40);
        //mWeightShifter.setInverted(true);
        //mWeightShifter.setSoftLimit(SoftLimitDirection.kForward, 27);
        //mWeightShifter.setSoftLimit(SoftLimitDirection.kReverse, -1);
        //mWeightPID.setSmartMotionMaxAccel(22000, 0);
        //mWeightPID.setSmartMotionMaxVelocity(6000, 0);
        //mWeightPID.setP(0.00002499999936844688, 0);
        //mWeightPID.setFF(1/6000, 0);
        //PIDController WeightPID = new PIDController(0.0, 0.0, 0.0);
        


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
                mLeftLeader.set(periodicIO.leftPercent);
                mRightLeader.set(periodicIO.rightPercent);
                //mWeightShifter.set(periodicIO.weightSpeed);
                //System.out.println(periodicIO.leftPercent);
                //System.out.println(periodicIO.rightPercent);
                break;  

            case PATH_FOLLOWING:
                //mPIDControllerL.setFF((periodicIO.leftFF));
                //mPIDControllerR.setFF((periodicIO.rightFF));
                //System.out.println("ioff: " + (float)(periodicIO.leftFF));
                //System.out.println("ioff: " + (float)(periodicIO.rightFF));
                //mPIDControllerL.setFF(ChassisConstants.kV);
                //mPIDControllerR.setFF(ChassisConstants.kV);
                mPIDControllerL.setReference(periodicIO.desiredLeftVelocity, ControlType.kVelocity,0,periodicIO.leftFF);
                mPIDControllerR.setReference(periodicIO.desiredRightVelocity, ControlType.kVelocity,0,periodicIO.rightFF);
                SmartDashboard.putNumber("left input RPM", periodicIO.desiredLeftVelocity*ChassisConstants.MPSToRPM);
                SmartDashboard.putNumber("right input RPM", periodicIO.desiredRightVelocity*ChassisConstants.MPSToRPM);
                SmartDashboard.putNumber("left applied output", mLeftLeader.getAppliedOutput());
                SmartDashboard.putNumber("left ouput current", mLeftLeader.getOutputCurrent());
                SmartDashboard.putNumber("right applied output", mRightLeader.getAppliedOutput());
                SmartDashboard.putNumber("right ouput current", mRightLeader.getOutputCurrent());
                SmartDashboard.putNumber("desired left vel m/s" , periodicIO.desiredLeftVelocity * 0.5);
                SmartDashboard.putNumber("desired right vel m/s" , periodicIO.desiredRightVelocity * 0.5);
                break;
        }
    }

    @Override
    public void update(double timestamp) {
        SmartDashboard.putNumber("encoderL pos", encoderL.getPosition());
        SmartDashboard.putNumber("encoderR pos", encoderR.getPosition());

        // Autobalance
        periodicIO.pitch = mImu.getPitch();
        
        periodicIO.leftPosition = encoderL.getPosition() * ChassisConstants.kEncoderDistancePerPulse;
        periodicIO.rightPosition = encoderR.getPosition() * ChassisConstants.kEncoderDistancePerPulse;

        // velocity
        periodicIO.actualLeftVelocity = encoderL.getVelocity() * ChassisConstants.kEncoderDistancePerPulse;
        periodicIO.actualRightVelocity = encoderR.getVelocity() * ChassisConstants.kEncoderDistancePerPulse;

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

        SmartDashboard.putNumber("left enc vel", encoderL.getVelocity());
        SmartDashboard.putNumber("Right enc vel", encoderR.getVelocity());

        SmartDashboard.putNumber("desired vel", -periodicIO.desiredLeftVelocity);

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
            setVelocity(leftVel, rightVel);
        }
    }

    /** Stops the compressor and all chassis motors */
    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;

        periodicIO.leftPercent = 0.0;
        periodicIO.rightPercent = 0.0;

        mLeftLeader.set(0.0);
        mRightLeader.set(0.0);
        setBrakeMode(true);
    }

    /**
     * Desired position for the weigh to go to using SmartMotion
     * 
     * @param position Position of weight [-1, 27]
     */
    // public void setWeightPos(boolean reverse) {
        //mWeightPID.setReference(position, ControlType.kSmartMotion);
        // if(reverse==false) mWeightShifter.set(1);
        // else mWeightShifter.set(-1);
    // }
// 
    // public double getWeightPos(){
        // return mWeightEncoder.getPosition();
    // }

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

    public double getFilter(double input){
        return driveFilter.calculate(input);
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
        //throttle = driveFilter.calculate(throttle); //Math.signum(throttle) * Math.pow(throttle, 2);
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
        encoderL.setPosition(0);
        encoderR.setPosition(0);
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
        if (brake) {
            mRightLeader.setIdleMode(IdleMode.kBrake);
            mLeftLeader.setIdleMode(IdleMode.kBrake);
            mRightFollowerA.setIdleMode(IdleMode.kBrake);
            mRightFollowerB.setIdleMode(IdleMode.kBrake);
            mLeftFollowerA.setIdleMode(IdleMode.kBrake);
            mLeftFollowerB.setIdleMode(IdleMode.kBrake);

        } else {
            mRightLeader.setIdleMode(IdleMode.kCoast);
            mLeftLeader.setIdleMode(IdleMode.kCoast);
            mRightFollowerA.setIdleMode(IdleMode.kCoast);
            mRightFollowerB.setIdleMode(IdleMode.kCoast);
            mLeftFollowerA.setIdleMode(IdleMode.kCoast);
            mLeftFollowerB.setIdleMode(IdleMode.kCoast);
        }
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
}
