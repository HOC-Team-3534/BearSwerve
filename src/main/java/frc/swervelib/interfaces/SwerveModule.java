package frc.swervelib.interfaces;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.swervelib.helpers.ModuleConfiguration;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    ModuleConfiguration getModuleConfiguration();

    DriveController getDriveController();

    SteerController getSteerController();

    AbsoluteEncoder getAbsoluteEncoder();

    void resetWheelEncoder();

    void set(double driveVoltage, double steerAngle);

    void setVelocity(double driveVelocity, double steerAngle);

    SwerveModulePosition getPosition();
}
