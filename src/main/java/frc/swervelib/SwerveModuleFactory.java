package frc.swervelib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.swervelib.helpers.ModuleConfiguration;
import frc.swervelib.interfaces.AbsoluteEncoder;
import frc.swervelib.interfaces.DriveController;
import frc.swervelib.interfaces.SteerController;
import frc.swervelib.interfaces.SwerveModule;
import frc.swervelib.interfaces.functional.DriveControllerFactory;
import frc.swervelib.interfaces.functional.SteerControllerFactory;

public class SwerveModuleFactory<DriveConfiguration, SteerConfiguration> {
    private final ModuleConfiguration moduleConfiguration;
    private final DriveControllerFactory<?, DriveConfiguration> driveControllerFactory;
    private final SteerControllerFactory<?, SteerConfiguration> steerControllerFactory;

    public SwerveModuleFactory(ModuleConfiguration moduleConfiguration,
                               DriveControllerFactory<?, DriveConfiguration> driveControllerFactory,
                               SteerControllerFactory<?, SteerConfiguration> steerControllerFactory) {
        this.moduleConfiguration = moduleConfiguration;
        this.driveControllerFactory = driveControllerFactory;
        this.steerControllerFactory = steerControllerFactory;
    }

    public SwerveModule create(DriveConfiguration driveConfiguration,
                               SteerConfiguration steerConfiguration, String namePrefix) {
        var driveController = driveControllerFactory.create(driveConfiguration, moduleConfiguration);
        var steerController = steerControllerFactory.create(steerConfiguration, moduleConfiguration);
        return new ModuleImplementation(driveController, steerController, namePrefix);
    }

    public SwerveModule create(ShuffleboardLayout container, DriveConfiguration driveConfiguration,
                               SteerConfiguration steerConfiguration, String namePrefix) {
        var driveController = driveControllerFactory.create(container, driveConfiguration, moduleConfiguration);
        var steerContainer = steerControllerFactory.create(container, steerConfiguration, moduleConfiguration);
        return new ModuleImplementation(driveController, steerContainer, namePrefix);
    }

    private class ModuleImplementation implements SwerveModule {
        private final DriveController driveController;
        private final SteerController steerController;
        private ShuffleboardTab tab = Shuffleboard.getTab("SwerveDt");
        private GenericEntry driveVoltageCmdEntry;
        private GenericEntry driveVelocityCmdEntry;
        private GenericEntry steerAngleCmdEntry;

        private ModuleImplementation(DriveController driveController,
                                     SteerController steerController, String namePrefix) {
            this.driveController = driveController;
            this.steerController = steerController;
            this.driveVoltageCmdEntry = tab.add(namePrefix + "Wheel Voltage Cmd V", 0).getEntry();
            this.driveVelocityCmdEntry = tab.add(namePrefix
                                                 + "Wheel Velocity Cmd RPM", 0).getEntry();
            this.steerAngleCmdEntry = tab.add(namePrefix + "Azmth Des Angle Deg", 0).getEntry();
        }

        @Override
        public void resetWheelEncoder() {
            driveController.resetEncoder();
        }

        @Override
        public double getDriveVelocity() {
            return driveController.getStateVelocity();
        }

        @Override
        public Rotation2d getSteerAngle() {
            return Rotation2d.fromRadians(MathUtil.angleModulus(steerController.getStateAngle().getRadians()));
        }

        @Override
        public ModuleConfiguration getModuleConfiguration() {
            return moduleConfiguration;
        }

        @Override
        public DriveController getDriveController() {
            return driveController;
        }

        @Override
        public SteerController getSteerController() {
            return steerController;
        }

        @Override
        public AbsoluteEncoder getAbsoluteEncoder() {
            return steerController.getAbsoluteEncoder();
        }

        @Override
        public void set(double driveVoltage, Rotation2d steerAngle) {
            if (isLongRotation(steerAngle)) {
                steerAngle.plus(Rotation2d.fromDegrees(180));
                driveVoltage *= -1.0;
            }
            driveController.setReferenceVoltage(driveVoltage);
            steerController.setReferenceAngle(steerAngle);
            this.driveVoltageCmdEntry.setDouble(driveVoltage);
            this.steerAngleCmdEntry.setDouble(steerAngle.getDegrees());
        }

        @Override
        public void setVelocity(double driveVelocity, Rotation2d steerAngle) {
            if (isLongRotation(steerAngle)) {
                steerAngle.plus(Rotation2d.fromDegrees(180));
                driveVelocity *= -1.0;
            }
            driveController.setVelocity(driveVelocity);
            steerController.setReferenceAngle(steerAngle);
            this.driveVelocityCmdEntry.setDouble(driveVelocity);
            this.steerAngleCmdEntry.setDouble(steerAngle.getDegrees());
        }

        private boolean isLongRotation(Rotation2d steerAngle) {
            return Math.abs(steerAngle.minus(getSteerAngle()).getDegrees()) > 90;
        }

        @Override
        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(getDriveController().getDistanceMeters(),
                                            getAbsoluteEncoder().getAbsoluteAngle());
        }
    }
}
