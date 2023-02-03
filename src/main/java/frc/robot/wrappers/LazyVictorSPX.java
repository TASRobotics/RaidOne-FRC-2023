package frc.robot.wrappers;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class LazyVictorSPX extends VictorSPX {
    protected double prevVal = 0;
    protected ControlMode prevControlMode = null;

    public LazyVictorSPX(int deviceNumber) {
        super(deviceNumber);
    }

    public double getLastSet() {
        return prevVal;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != prevVal || mode != prevControlMode) {
            prevVal = value;
            prevControlMode = mode;
            super.set(mode, value);
        }
    }
}
