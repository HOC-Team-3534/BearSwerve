package frc.swervelib.helpers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.swervelib.interfaces.Gyroscope;
import frc.swervelib.vendors.ctre.PigeonFactoryBuilder;
import frc.swervelib.vendors.kauailabs.navXFactoryBuilder;

public final class GyroscopeHelper {
    private GyroscopeHelper() {
    }

    public static Gyroscope createPigeonController(TalonSRX controller) {
        WPI_PigeonIMU pigeon = new WPI_PigeonIMU(controller);
        return new PigeonFactoryBuilder().build(pigeon);
    }

    public static Gyroscope createPigeonCAN(Integer id) {
        WPI_PigeonIMU pigeon = new WPI_PigeonIMU(id);
        return new PigeonFactoryBuilder().build(pigeon);
    }

    public static Gyroscope createPigeon2CAN(Integer id) {
        WPI_Pigeon2 pigeon = new WPI_Pigeon2(id);
        return new PigeonFactoryBuilder().build(pigeon);
    }

    public static Gyroscope createnavXMXP() {
        AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200);
        return new navXFactoryBuilder().build(navx);
    }

    public static Gyroscope createnavXUSB() {
        AHRS navx = new AHRS(SerialPort.Port.kUSB, SerialDataType.kProcessedData, (byte) 200);
        return new navXFactoryBuilder().build(navx);
    }
}
