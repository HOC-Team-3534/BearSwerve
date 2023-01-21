package frc.swervelib.vendors.kauailabs;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.swervelib.interfaces.Gyroscope;

public class navXFactoryBuilder {
    public Gyroscope build(AHRS navX) {
        return new GyroscopeImplementation(navX);
    }

    private static class GyroscopeImplementation implements Gyroscope {
        private final AHRS navX;
        private final SimDouble angleSim;
        private static Rotation2d gyroOffset = new Rotation2d();

        private GyroscopeImplementation(AHRS navX) {
            this.navX = navX;
            int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            angleSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        }

        @Override
        public Rotation2d getGyroHeading() {
            if (navX.isMagnetometerCalibrated()) {
                // We will only get valid fused headings if the magnetometer is calibrated
                return Rotation2d.fromDegrees(navX.getFusedHeading()).plus(gyroOffset);
            }
            // We have to invert the angle of the NavX so that rotating the robot
            // counter-clockwise makes the angle increase.
            return navX.getRotation2d().plus(gyroOffset);
        }

        @Override
        public Boolean getGyroReady() {
            return !navX.isCalibrating();
        }

        @Override
        public void zeroGyroscope(Rotation2d angle) {
            gyroOffset = angle.minus(getGyroHeading());
        }

        @Override
        public void setAngleSim(Rotation2d angle) {
            angleSim.set(angle.getDegrees());
        }
    }
}
