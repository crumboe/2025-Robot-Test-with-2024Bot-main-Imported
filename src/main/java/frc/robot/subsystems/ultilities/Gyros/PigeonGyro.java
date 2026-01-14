package frc.robot.subsystems.ultilities.Gyros;

import com.ctre.phoenix6.hardware.Pigeon2;
// import frc.robot.subsystems.Gyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PigeonGyro extends SubsystemBase implements Gyro {

    private static PigeonGyro instance;

    public static Pigeon2 pigeon;
    public static double zeroHeading;
    public static double zeroAngle;

    /** Creates a new PigeonGyro. */
    private PigeonGyro() {
        pigeon = new Pigeon2(DriveConstants.PigeonCANId);

        zeroHeading = getGyroHeading();
        zeroAngle = getGyroAngle();
        // System.out.println("Setup ZeroAngle " + zeroAngle);
    }

    // Public Methods

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new PigeonGyro();
        }
        return instance;
    }

    @Override
    public double getGyroHeading() {
        double heading = pigeon.getYaw().getValueAsDouble();
        return heading;
    }

    @Override
    public double getGyroAngle() {
        double angle = pigeon.getYaw().getValueAsDouble();
        return angle;
    }

    @Override
    public void setGyroAngleOffset(double offset) {
        pigeon.setYaw(offset);
    }

    @Override
    public void zeroGyroHeading() {
        // navX.zeroYaw();
        pigeon.reset();
        zeroHeading = getGyroHeading();
        zeroAngle = getGyroAngle();
        System.out.println("ZeroHeading: " + zeroHeading);
        System.out.println("ZeroAngle: " + zeroAngle);
    }

    @Override
    public double getZeroHeading() {
        return zeroHeading;
    }

    @Override
    public double getZeroAngle() {
        return zeroAngle;
    }

    @Override
    public Rotation2d getGyroRotation2D() {
        return pigeon.getRotation2d();
        // return Rotation2d.fromDegrees(getHeading());
    }

    /*
     * Note that the math in the getHeading method is used to invert the direction
     * of
     * the gyro for use by wpilib which treats gyros backwards.
     * Gyros are normally clockwise positive. Wpilib wants
     * counter-clockwise positive.
     */
    @Override
    public double getHeading() {
        // return Math.IEEEremainder(-getNavAngle(), 360);
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public double getPitchAngle() {
        return pigeon.getPitch().getValueAsDouble();
    }
}