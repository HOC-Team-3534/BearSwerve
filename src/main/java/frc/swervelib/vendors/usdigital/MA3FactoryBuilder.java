package frc.swervelib.vendors.usdigital;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.swervelib.interfaces.AbsoluteEncoder;
import frc.swervelib.interfaces.functional.AbsoluteEncoderFactory;

public class MA3FactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;
    private static MA3AbsoluteConfiguration configuration;
    private static double angle = 0;

    public MA3FactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public AbsoluteEncoderFactory<MA3AbsoluteConfiguration> build() {
        return configuration -> {
            this.configuration = configuration;
            AnalogInput encoder = new AnalogInput(configuration.getId());
            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final AnalogInput encoder;

        private EncoderImplementation(AnalogInput encoder) {
            this.encoder = encoder;
        }

        @Override
        public Rotation2d getAbsoluteAngle() {
            angle = (1.0 - encoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;
            angle += configuration.getOffset();
            return Rotation2d.fromRadians(angle);
        }

        @Override
        public Rotation2d getAbsoluteAngleRetry() {
            // No communication error to
            return getAbsoluteAngle();
        }

        @Override
        public void setAbsoluteEncoder(Rotation2d position, double velocity) {
            // encoder.setSimDevice(device);
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
