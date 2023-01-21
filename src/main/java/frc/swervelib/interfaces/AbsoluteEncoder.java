package frc.swervelib.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AbsoluteEncoder {
    /**
     * Gets the current angle reading of the encoder.
     *
     * @return The current angle
     */
    Rotation2d getAbsoluteAngle();

    Rotation2d getAbsoluteAngleRetry();

    void setAbsoluteEncoder(Rotation2d position, double velocity);
}
