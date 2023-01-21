package frc.swervelib.interfaces.functional;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.swervelib.helpers.ModuleConfiguration;
import frc.swervelib.interfaces.DriveController;

@FunctionalInterface
public interface DriveControllerFactory<Controller extends DriveController, DriveConfiguration> {
    default void addDashboardEntries(ShuffleboardContainer container, Controller controller) {
        container.addNumber("Current Velocity", controller::getStateVelocity);
    }

    default Controller create(ShuffleboardContainer container,
                              DriveConfiguration driveConfiguration,
                              ModuleConfiguration moduleConfiguration) {
        var controller = create(driveConfiguration, moduleConfiguration);
        // addDashboardEntries(container, controller);
        return controller;
    }

    Controller create(DriveConfiguration driveConfiguration,
                      ModuleConfiguration moduleConfiguration);
}
