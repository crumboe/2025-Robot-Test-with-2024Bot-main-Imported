// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.ultilities.Gyros;

import edu.wpi.first.math.geometry.Rotation2d;

/*
 * Interface Gyro is a template for creating the gyro classes pigeongyro and navxgryo, and any 
 * future gyro types. This interface lets us define these seperate classes as the same "type"
 * everywhere else in the code, and call the functions without caring which one we are using.
 * This is more expandable and easy to change over time (hopefully, otherwise that is totally my mistake CMR)
 * 
 */
public interface Gyro {
    public double getGyroHeading();

    public double getGyroAngle();

    public void setGyroAngleOffset(double offset);

    public void zeroGyroHeading();

    public double getZeroHeading();

    public double getZeroAngle();

    public Rotation2d getGyroRotation2D();

    public double getHeading();

    public Rotation2d getRotation2d();

    public double getPitchAngle();

}
