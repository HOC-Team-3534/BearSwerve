package frc.swervelib.interfaces.functional;

import frc.swervelib.interfaces.AbsoluteEncoder;

@FunctionalInterface
public interface AbsoluteEncoderFactory<Configuration> {
    AbsoluteEncoder create(Configuration configuration);
}
