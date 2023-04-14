package frc.robot.submodules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WeightConstants;

public class WeightShifter extends Submodule{

    private CANSparkMax mWeightShifter = new CANSparkMax(WeightConstants.WEIGHTSHIFTER_ID, MotorType.kBrushless);
    private SparkMaxPIDController mWeightPID;
    private RelativeEncoder mWeightEncoder;
    public double desiredVel = 0;
    
    private WeightShifter() {}
    private static WeightShifter instance = null;
    public static WeightShifter getInstance() {
        if (instance == null) {
            instance = new WeightShifter();
        }
        return instance;
    }
    
    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    @Override
    public void onStart(double timestamp) {
        //stop();
        mWeightShifter.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Called once when the submodule is initialized.
     */
    @Override
    public void onInit() {
        mWeightShifter.restoreFactoryDefaults();

        mWeightPID = mWeightShifter.getPIDController();
        mWeightEncoder = mWeightShifter.getEncoder();

        mWeightShifter.setIdleMode(IdleMode.kBrake);
        mWeightShifter.setSmartCurrentLimit(40);
        mWeightShifter.setInverted(false);
        mWeightShifter.setSoftLimit(SoftLimitDirection.kForward, (float) 15.5);
        mWeightShifter.setSoftLimit(SoftLimitDirection.kReverse, -1);
        mWeightPID.setSmartMotionMaxVelocity(WeightConstants.MAX_VELOCITY, WeightConstants.SMART_MOTION_ID);
        mWeightPID.setSmartMotionMaxAccel(WeightConstants.MAX_ACCEL, WeightConstants.SMART_MOTION_ID);
        mWeightPID.setP(WeightConstants.kP, WeightConstants.SMART_MOTION_ID);
        mWeightPID.setFF(WeightConstants.kF, WeightConstants.SMART_MOTION_ID);
        mWeightPID.setP(WeightConstants.POS_kP, WeightConstants.POSITION_ID);
        mWeightPID.setFF(WeightConstants.POS_kF, WeightConstants.POSITION_ID);
        mWeightPID.setP(WeightConstants.POS_kP_HOLD, WeightConstants.POSITION_ID_HOLD);
        mWeightPID.setFeedbackDevice(mWeightEncoder);
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    public void update(double timestamp) {
        SmartDashboard.putNumber("Weight desiredvel", desiredVel);
        SmartDashboard.putNumber("Weight position", getWeightPos());
    }
    
    /**
     * Runs components in the submodule that have continuously changing 
     * inputs.
     */
    public void run() {
        mWeightShifter.set(desiredVel);
    }

    /**
     * Stops the submodule.
     */
    @Override
    public void stop() {
        reset();
        mWeightShifter.setIdleMode(IdleMode.kCoast);
        //mWeightShifter.set(0);
    }

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {}

    /**
     * Resets weight shifter to position 0
     */
    public void reset() {
        mWeightPID.setReference(WeightConstants.WEIGHT_REAR, ControlType.kSmartMotion, WeightConstants.SMART_MOTION_ID);
    }
    
    /**
     * @return Returns the position of weight shifter
     */
    public double getWeightPos() {
        return mWeightEncoder.getPosition();
    }

    /**
     * Moves the weight shifter to desired position
     * @param position Value of desired position [-1, 27]
     */
    public void setPosition(double position) {
        mWeightPID.setReference(position, ControlType.kSmartMotion, WeightConstants.POSITION_ID_HOLD);
        System.out.println("Holding weight");
    }

    /**
     * Moves the weight shifter at a desired velocity
     * @param velocity Value of desired velocity [-1, 1]
     */
    public void setVelocity(double velocity) {
        desiredVel = velocity;
    }

    /**
     * Moves the weight block at a high velocity for inertia
     */
    public void punch() {
        
    }

}