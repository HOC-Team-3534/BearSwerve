package frc.swervelib.vendors.ctre;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.swervelib.interfaces.Gyroscope;

public class PigeonFactoryBuilder {
    private static BasePigeonSimCollection pigeonSim;
    private static Rotation2d gyroOffset = new Rotation2d();

    public Gyroscope build(WPI_PigeonIMU pigeon) {
        return new GyroscopeImplementation(pigeon);
    }

    public Gyroscope build(WPI_Pigeon2 pigeon) {
        return new GyroscopeImplementation2(pigeon);
    }

    private static class GyroscopeImplementation implements Gyroscope {
        private final WPI_PigeonIMU pigeon;

        private GyroscopeImplementation(WPI_PigeonIMU pigeon) {
            this.pigeon = pigeon;
            pigeonSim = pigeon.getSimCollection();
        }

        @Override
        public Rotation2d getGyroHeading() {
            return Rotation2d.fromDegrees(pigeon.getFusedHeading()).plus(gyroOffset);
        }

        @Override
        public Boolean getGyroReady() {
            return pigeon.getState().equals(PigeonState.Ready);
        }

        @Override
        public void zeroGyroscope(Rotation2d angle) {
            gyroOffset = angle.minus(getGyroHeading());
        }

        @Override
        public void setAngleSim(Rotation2d angle) {
            pigeonSim.setRawHeading(angle.getDegrees());
        }
    }

    private static class GyroscopeImplementation2 implements Gyroscope {
        private final WPI_Pigeon2 pigeon;

        private GyroscopeImplementation2(WPI_Pigeon2 pigeon) {
            this.pigeon = pigeon;
            pigeonSim = pigeon.getSimCollection();
        }

        @Override
        public Rotation2d getGyroHeading() {
            return pigeon.getRotation2d().plus(gyroOffset);
        }

        @Override
        public Boolean getGyroReady() {
            return true; // TODO determine when the pigeon 2
        }

        @Override
        public void zeroGyroscope(Rotation2d angle) {
            gyroOffset = angle.minus(getGyroHeading());
        }

        @Override
        public void setAngleSim(Rotation2d angle) {
            pigeonSim.setRawHeading(angle.getDegrees());
        }
    }
}
