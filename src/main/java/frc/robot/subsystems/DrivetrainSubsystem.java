package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    
    /**
     * The maximum velocity of the robot in meters per second.
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double maxVelocity = 5676.0  / 60.0
    * SdsModuleConfigurations.MK4_L2.getDriveReduction()
    * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
    * Math.PI;
    
    
    /**
     * The maximum angular velocity of the robot in radians per second.
     * This is a measure of how fast the robot can rotate in place.
     * The calculation here is theoretical, but if we end up measuring it we can replace this number
     */
    public static final double maxAngularVelocity = maxVelocity /
    Math.hypot(trackWidth / 2.0, wheelbaseLength / 2.0);
    
    // our kinematics class to do all the driving calculations for us
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(trackWidth / 2.0, wheelbaseLength / 2.0),
        
        // Front right
        new Translation2d(trackWidth / 2.0, -wheelbaseLength / 2.0),
        
        // Back left
        new Translation2d(-trackWidth / 2.0, wheelbaseLength / 2.0),
        
        // Back right
        new Translation2d(-trackWidth / 2.0, -wheelbaseLength / 2.0)
    );
        
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should cause the angle reading to increase until it wraps back over to zero.
        // Our NavX2 IMU
        private final AHRS gyro = new AHRS(Port.kMXP, (byte) 200);
        
        // These are our modules. We initialize them in the constructor.
        private final SwerveModule frontLeftModule;
        private final SwerveModule frontRightModule;
        private final SwerveModule backLeftModule;
        private final SwerveModule backRightModule;
        
        // chassis speeds variable for our kinematics calculations
        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        private SwerveModuleState[] states;
        
        public DrivetrainSubsystem() {
            // Drivetrain shuffleboard tab for our dashboard
            ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
            
            frontLeftModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                
                // gear ratio
                Mk4SwerveModuleHelper.GearRatio.L2,
                
                // device ids
                frontLeftDriveMotor,
                frontLeftSteerMotor,
                frontLeftSteerEncoder,
                
                // This is how much the steer encoder is offset from true zero (In our case,
                // zero is facing straight forward)
                frontLeftModuleOffset
                );
                
                frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                    tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                    Mk4SwerveModuleHelper.GearRatio.L2,
                frontRightDriveMotor,
                frontRightSteerMotor,
                frontRightSteerEncoder,
                frontRightOffset
        );

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                backLeftDriveMotor,
                backLeftSteerMotor,
                backLeftSteerEncoder,
                backLeftOffset
        );

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                backLeftDriveMotor,
                backLeftSteerMotor,
                backLeftSteerEncoder,
                backLeftOffset
        );
    }

    /**
     * Reset ("zero") our NavX2 IMU
     */
    public void resetGyro() {
        gyro.zeroYaw();
    }

    /**
     * Get the gyroscope heading
     * @return the heading of the robot, as a Rotation2d object, with the angle inverted for the swerve kinematics
     */
    public Rotation2d getGyroscopeRotation() {
        // if the gyroscope is calibrated
        if (gyro.isMagnetometerCalibrated()) {
            // then we can get the fused heading
            return Rotation2d.fromDegrees(360 - gyro.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    }

    /**
     * The actual drive method. This method doesn't make the motors move,
     * it just sets the desired speeds which are then turned into control outputs by our periodic()
     * @param speeds
     */
    public void drive(ChassisSpeeds speeds) {
        chassisSpeeds = speeds;
    }

    /**
     * Runs once every main loop iteration
     * Responsible for actually issuing the control outputs to our motors
     */
    @Override
    public void periodic() {
        states = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);

        frontLeftModule.set(states[0].speedMetersPerSecond / maxVelocity * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / maxVelocity * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / maxVelocity * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / maxVelocity * MAX_VOLTAGE, states[3].angle.getRadians());
    }
}
