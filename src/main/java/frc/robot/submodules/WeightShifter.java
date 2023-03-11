package frc.robot.submodules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.WeightConstants;

public class WeightShifter extends Submodule{

    private final CANSparkMax mWeightShifter;
    private SparkMaxPIDController mWeightPID;
    private RelativeEncoder mWeightEncoder;
    
    public WeightShifter(int ID, MotorType type) {
        mWeightShifter = new CANSparkMax(ID, type);
    };
    
    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    public void onStart(double timestamp) {
        stop();
    }

    /**
     * Called once when the submodule is initialized.
     */
    public void onInit() {
        mWeightShifter.restoreFactoryDefaults();

        mWeightPID = mWeightShifter.getPIDController();
        mWeightEncoder = mWeightShifter.getEncoder();

        mWeightShifter.setIdleMode(IdleMode.kBrake);
        mWeightShifter.setSmartCurrentLimit(40);
        mWeightShifter.setInverted(false);
        mWeightShifter.setSoftLimit(SoftLimitDirection.kForward, 27);
        mWeightShifter.setSoftLimit(SoftLimitDirection.kReverse, -1);
        mWeightPID.setSmartMotionMaxVelocity(WeightConstants.kMAX_VELOCITY, WeightConstants.SMART_MOTION_ID);
        mWeightPID.setSmartMotionMaxAccel(WeightConstants.kMAX_ACCEL, WeightConstants.SMART_MOTION_ID);
        mWeightPID.setP(WeightConstants.kP, WeightConstants.SMART_MOTION_ID);
        mWeightPID.setFF(WeightConstants.kF, WeightConstants.SMART_MOTION_ID);
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    public void update(double timestamp) {}
    
    /**
     * Runs components in the submodule that have continuously changing 
     * inputs.
     */
    public void run() {}

    /**
     * Stops the submodule.
     */
    public void stop() {
        reset();
        mWeightShifter.set(0);
    }

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {}

    public void reset() {
        mWeightPID.setReference(WeightConstants.WEIGHT_REAR, ControlType.kSmartMotion);
    }

    public double getPosition() {
        return mWeightEncoder.getPosition();
    }

    public void setPosition(double position) {
        mWeightPID.setReference(position, ControlType.kSmartMotion);
    }

    public void setVelocity(double velocity) {
        mWeightShifter.set(velocity);
    }

    public void punch() {
        if (mWeightEncoder.getPosition() >= 26) {
            mWeightShifter.set(-0.8);
        } else if (mWeightEncoder.getPosition() <= 2) {
            mWeightShifter.set(0.8);
        }
    }

}