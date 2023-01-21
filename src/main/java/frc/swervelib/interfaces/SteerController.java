package frc.swervelib.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public interface SteerController {
    Rotation2d getReferenceAngle();

    void setReferenceAngle(Rotation2d referenceAngle);

    DCMotor getSteerMotor();

    AbsoluteEncoder getAbsoluteEncoder();

    Rotation2d getStateAngle();

    double getOutputVoltage();

    void setSteerEncoder(double position, double velocity);
}
