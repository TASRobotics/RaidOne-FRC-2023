package frc.robot.wrappers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class LogicalTalonSRX extends WPI_TalonSRX {
    public LogicalTalonSRX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public double getSelectedSensorVelocity() {
        return super.getSelectedSensorVelocity() * 10;
    }

    @Override
    public void set(ControlMode controlMode, double demand0, DemandType demandType, double demand1) {
        if(controlMode == ControlMode.Velocity) {
            super.set(controlMode, demand0 / 10, demandType, demand1 / 10);
        } else {
            super.set(controlMode, demand0, demandType, demand1);
        }
    }

    @Override
    public ErrorCode config_kP(int slotIdx, double value, int timeoutMS) {
        return super.config_kP(slotIdx, value / 10, timeoutMS);
    }

    @Override
    public ErrorCode config_kI(int slotIdx, double value, int timeoutMS) {
        return super.config_kI(slotIdx, value / 10, timeoutMS);
    }

    @Override
    public ErrorCode config_kD(int slotIdx, double value, int timeoutMS) {
        return super.config_kD(slotIdx, value / 10, timeoutMS);
    }

    
}