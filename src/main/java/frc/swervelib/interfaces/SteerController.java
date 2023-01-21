package frc.swervelib.interfaces;

import edu.wpi.first.math.system.plant.DCMotor;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    DCMotor getSteerMotor();

    AbsoluteEncoder getAbsoluteEncoder();

    double getStateAngle();

    double getOutputVoltage();

    void setSteerEncoder(double position, double velocity);
}
