package frc.robot.wrappers;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Constants.ChassisConstants;

public class InactiveCompressor extends Compressor {

    private static InactiveCompressor instance = null;
    
    private boolean state = true;

    public static InactiveCompressor getInstance() {
        if (instance == null) {
            instance = new InactiveCompressor();
        }
        return instance;
    }

    public InactiveCompressor() {
        super(ChassisConstants.PNEUMATICS_MODULE_TYPE);
    }

    public boolean getState() {
        return state;
    }

    public void changeState() {
        state = !state;
        if (state) {
            super.enableDigital();
        } else {
            super.disable();
        }
    }
}