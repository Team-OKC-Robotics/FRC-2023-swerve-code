package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double trackWidth = 1.0; // FIXME Measure and set trackwidth
  
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double wheelbaseLength = 1.0; // FIXME Measure and set wheelbase

    /**
     * Front left module
     */
    public static final int frontLeftDriveMotor = 1;
    public static final int frontLeftSteerMotor = 2;
    public static final int frontLeftSteerEncoder = 0; // FIXME Set front left steer encoder ID
    public static final double frontLeftModuleOffset = -Math.toRadians(0.0); // FIXME Measure and set front left steer offset

    /**
     * Front right module
     */
    public static final int frontRightDriveMotor = 3;
    public static final int frontRightSteerMotor = 4;
    public static final int frontRightSteerEncoder = 0; // FIXME Set front right steer encoder ID
    public static final double frontRightOffset = -Math.toRadians(0.0); // FIXME Measure and set front right steer offset

    /**
     * Back left module
     */
    public static final int backLeftDriveMotor = 5;
    public static final int backLeftSteerMotor = 6;
    public static final int backLeftSteerEncoder = 0; // FIXME Set back left steer encoder ID
    public static final double backLeftOffset = -Math.toRadians(0.0); // FIXME Measure and set back left steer offset

    /**
     * Back right module
     */
    public static final int backRightDriveMotor = 7;
    public static final int backRightSteerMotor = 8;
    public static final int backRightSteerEncoder = 0; // FIXME Set back right steer encoder ID
    public static final double backRightOffset = -Math.toRadians(0.0); // FIXME Measure and set back right steer offset
}
