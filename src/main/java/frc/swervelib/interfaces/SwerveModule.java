package frc.swervelib.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.swervelib.helpers.ModuleConfiguration;

public interface SwerveModule {
    double getDriveVelocity();

    Rotation2d getSteerAngle();

    ModuleConfiguration getModuleConfiguration();

    DriveController getDriveController();

    SteerController getSteerController();

    AbsoluteEncoder getAbsoluteEncoder();

    void resetWheelEncoder();

    void set(double driveVoltage, Rotation2d steerAngle);

    void setVelocity(double driveVelocity, Rotation2d steerAngle);

    SwerveModulePosition getPosition();
}
