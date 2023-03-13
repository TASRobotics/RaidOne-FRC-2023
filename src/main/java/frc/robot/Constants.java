package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {
    public static final class ChassisConstants {
        /** Motors */
        public static final int LEFT_LEADER_ID = 2;
        public static final int LEFT_FOLLOWER_A_ID = 4;
        public static final int LEFT_FOLLOWER_B_ID = 6;

        public static final int RIGHT_LEADER_ID = 1;
        public static final int RIGHT_FOLLOWER_A_ID = 3;
        public static final int RIGHT_FOLLOWER_B_ID = 5;

        public static final int WEIGHTSHIFTER_ID = 60;

        /** Sensors */
        public static final int IMU_ID = 0;

        /** Velocity PID */
        public static final int PID_LOOP_IDX = 0;
        public static final double kS = 0.0;
        public static final double kV = 1.5;
        //1/(6000/60*0.04965108462); around 0.2 smth
        public static final double kA = 0; //0.0003; //14.0
        public static final double kP = 0.0000; //0.3 //0.000025

        /** Slew rate limiter for drive accel */
        public static final double SLEW_FILTER = 0.5;
        
        /** Drive kinematics (for ramsete) */
        public static final double TRACK_WIDTH = 0.58; //! tune
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
            new DifferentialDriveKinematics(TRACK_WIDTH);

        public static final int ENCODER_CPR = 4096; // counts per rotation 8129
        public static final double WHEEL_DIAMETER = 0.1016; // meters //0.1016 //0.1524
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
          // (WHEEL_DIAMETER * Math.PI) / (double) ENCODER_CPR;
          1/4096*12/60*24/32/24*3.14159*0.1524; //! regearing
        
        public static final double kEncoderDistancePerRevolution =
        12/60*24/32/24*3.14159*0.1524; //! regearing

        public static final double MPSToRPM = 60.0/(kEncoderDistancePerRevolution);
            
        /** Teleop Constants */
        public static final int MONOMIAL_SCALE = 0;
        public static final double RAMP_RATE = 0.35;
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
        public static final double AUTOBAL_MULTIPLIER = 0.008; //0.006;

        //Autobalance PID constants
        public static final double AUTOBALPID_KFF = 0.000166;
        public static final double AUTOBALPID_KP = 0;
        public static final double AUTOBALPID_KI = 0;
        public static final double AUTOBALPID_KD = 0;
        public static final double AUTOBALPID_KIZONE = 0;
        public static final double AUTOBALPID_KMAXOUTPUT = 1;
        public static final double AUTOBALPID_KMINOUTPUT = -1;

    }

    public static final class WeightConstants {

        /** Weightshifter motor ID */
        public static final int WEIGHTSHIFTER_ID = 60;

        /** Weightshifter position constants */
        public static final double WEIGHT_FRONT = 27.0;
        public static final double WEIGHT_CENTER = 13.0;
        public static final double WEIGHT_REAR = -1.0;

        /** Weighshifter PID constants (Smartmotion) */
        public static final int SMART_MOTION_ID = 0;
        public static final double kP = 0.00002499999936844688;
        public static final double kF = 1/6000;
        public static final double MAX_VELOCITY = 6000.0;
        public static final double MAX_ACCEL = 22000.0;

        /** Weightshifter PID constants (Position) */
        public static final int POSITION_ID = 1;
        public static final double POS_kP = 0.0;
        public static final double POS_kF = 0.0;
    }
    
    /** Universal constants */
    public static final double DEADBAND = 0.06;
    public static final int TIMEOUT_MS = 10;
    public static final double VOLTAGE_COMPENSATION = 12.0;
}
