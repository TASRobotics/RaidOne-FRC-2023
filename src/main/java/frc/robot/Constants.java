package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {
    public static final class ChassisConstants {
        /** Motors */
        public static final int LEFT_LEADER_ID = 2;
        public static final int LEFT_FOLLOWER_A_ID = 3;
        public static final int LEFT_FOLLOWER_B_ID = 3;

        public static final int RIGHT_LEADER_ID = 1;
        public static final int RIGHT_FOLLOWER_A_ID = 4;
        public static final int RIGHT_FOLLOWER_B_ID = 13;

        /** Sensors */
        public static final int IMU_ID = 0;

        /** Pneumatics */
        public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
        public static final int SHIFTER_HIGH_TORQUE_ID = 7;
        public static final int SHIFTER_LOW_TORQUE_ID = 8;

        /** Velocity PID */
        public static final int PID_LOOP_IDX = 0;
        public static final double kS = 0.0;
        public static final double kV = 0.00016666666488163173; //2.6 units prob wrong
        public static final double kA = 0.0003; //14.0
        public static final double kP = 0.00003; //0.3
        
        /** Drive kinematics (for ramsete) */
        public static final double TRACK_WIDTH = 0.69;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
            new DifferentialDriveKinematics(TRACK_WIDTH);

        public static final int ENCODER_CPR = 4096; // counts per rotation 8129
        public static final double WHEEL_DIAMETER = 0.1016; // meters
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
          // (WHEEL_DIAMETER * Math.PI) / (double) ENCODER_CPR;
          1/4096*12/72*56/60*3.14159*0.1016;

        public static final double kEncoderDistancePerRevolution =
        12/72*56/60*3.14159*0.1016;
            
        /** Teleop Constants */
        public static final int MONOMIAL_SCALE = 0;
        public static final double RAMP_RATE = 0.5; // 0.33 //0.3
    }

    public static final class IntakeConstants {
        public static final int LEFT_LEADER_ID = 1;
        public static final int RIGHT_FOLLOWER_ID = 2;

        public static final int SOLENOID_DOWN_ID = 6;
        public static final int SOLENOID_UP_ID = 9;
    }

    public static final class EZClimbConstants {
        public static final int LEFT_ID = 21;
        public static final int RIGHT_ID = 22;

        public static final int SOLENOID_DOWN_ID = 5;
        public static final int SOLENOID_UP_ID = 10;
    }

    public static final class OIConstants {
        /** Controller constants */
        public static final int DRIVE_CONTROLLER_PORT = 0;
    }

    public static final class AutoConstants {
        /** Motion profile constants */
        public static final double MAX_VEL = 2; // meters per second
        public static final double MAX_ACCEL = 2; // meters per second ^ 2
        
        /** Ramsete constants */
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

        /** Autobalance multiplier */
        //* Charge station max tilt = 15 degrees
        public static final double AUTOBAL_MULTIPLIER = 0.025;

    }
    
    /** Universal constants */
    public static final double DEADBAND = 0.06;
    public static final int TIMEOUT_MS = 10;
    public static final double VOLTAGE_COMPENSATION = 12.0;
}
