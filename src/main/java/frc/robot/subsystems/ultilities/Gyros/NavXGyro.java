// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ultilities.Gyros;

// import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class NavXGyro extends SubsystemBase implements Gyro {

    private static NavXGyro instance;
    public static AHRS navX;
    public static double zeroHeading;
    public static double zeroAngle;

    /** Creates a new NavXGyro. */
    private NavXGyro() {
        navX = new AHRS(NavXComType.kMXP_SPI);

        zeroHeading = getGyroHeading();
        zeroAngle = getGyroAngle();
        // System.out.println("Setup ZeroAngle " + zeroAngle);
    }

    // Public Methods

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new NavXGyro();
        }
        return instance;
    }

    @Override
    public double getGyroHeading() {
        double heading = -navX.getFusedHeading();
        return heading;
    }

    @Override
    public double getGyroAngle() {
        double angle = -navX.getAngle();
        return angle;
    }

    @Override
    public void setGyroAngleOffset(double offset) {
        navX.setAngleAdjustment(offset);
    }

    @Override
    public void zeroGyroHeading() {
        // navX.zeroYaw();
        navX.reset();
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
        // return Rotation2d.fromDegrees(navX.getAngle());
        return Rotation2d.fromDegrees(-navX.getAngle());
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
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public double getPitchAngle() {
        return navX.getPitch();
    }
}