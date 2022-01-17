package frc.robot.Utilities.Drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Utilities.Constants;


public class CANSpeedControllerBuilder {
    private static class Configuration {
		public double MAX_OUTPUT = 1;
		public double NOMINAL_OUTPUT = 0;
		public IdleMode NEUTRAL_MODE = IdleMode.kBrake;
		public boolean ENABLE_CURRENT_LIMIT = false;
		public boolean ENABLE_SOFT_LIMIT = false;
		//public boolean ENABLE_LIMIT_SWITCH = false;
		public int CURRENT_LIMIT = 0;
		public boolean INVERTED = false;

		public int CONTROL_FRAME_PERIOD_MS = 10;
		public int STATUS_FRAME_GENERAL_1_MS = 10;
		public int STATUS_FRAME_FEEDBACK0_2_MS = 20;
		public int STATUS_FRAME_QUADRATURE_3_MS = 160;
		public int STATUS_FRAME_ANALOG_4_MS = 160;
		public int STATUS_FRAME_PULSE_8_MS = 160;
		public int STATUS_FRAME_TARGET_10_MS = 0;
		public int STATUS_FRAME_UART_11_MS = 250;
		public int STATUS_FRAME_FEEDBACK1_12_MS = 250;
		public int STATUS_FRAME_PIDF0_13_MS = 160;
		public int STATUS_FRAME_PIDF1_14_MS = 250;
		public int STATUS_FRAME_FIRMWARE_15_MS = 160;

		//public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod::Period_100Ms;
		//public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;
	}
	
	private static Configuration kDefaultConfiguration = new Configuration();
	private static Configuration kSlaveConfiguration = new Configuration();

    static {
		kSlaveConfiguration.STATUS_FRAME_GENERAL_1_MS = 100;
		kSlaveConfiguration.STATUS_FRAME_FEEDBACK0_2_MS = 100;
	}

    private CANSpeedControllerBuilder() { }

    public static CANSparkMax createDefaultSparkMax(int id) {
        return createSparkMax(id, kDefaultConfiguration);
    }

    public static SparkMaxU createDefaultSparkMax(int id, int pdpChannel) {
        return createSparkMax(id, pdpChannel, kDefaultConfiguration);
    }

    public static CANSparkMax createMasterSparkMax(int id) {
        Configuration masterConfig = new Configuration();
        return createSparkMax(id, masterConfig);
    }

    public static SparkMaxU createMasterSparkMax(int id, int pdpChannel) {
        Configuration masterConfig = new Configuration();
        return createSparkMax(id, pdpChannel, masterConfig);
    }

    public static CANSparkMax createFastMasterSparkMax(int id) {
        Configuration masterConfig = new Configuration();
        masterConfig.CONTROL_FRAME_PERIOD_MS = 5;
        masterConfig.STATUS_FRAME_GENERAL_1_MS = 5;

        return createSparkMax(id, masterConfig);
    }

    public static SparkMaxU createFastMasterSparkMax(int id, int pdpChannel) {
        Configuration masterConfig = new Configuration();
        masterConfig.CONTROL_FRAME_PERIOD_MS = 5;
        masterConfig.STATUS_FRAME_GENERAL_1_MS = 5;

        return createSparkMax(id, pdpChannel, masterConfig);
    }

    public static SparkMaxU createPermanentSlaveSparkMax(int id, CANSparkMax masterSpark) {
        SparkMaxU spark = createSparkMax(id, 0, kSlaveConfiguration);
        spark.follow(masterSpark);
        return spark;
    }

    public static CANSparkMax createSparkMax(int id, Configuration config) {
        CANSparkMax spark = new CANSparkMax(id, MotorType.kBrushless);
        configSpark(spark, config);
        return spark;
    }

    public static SparkMaxU createSparkMax(int id, int pdpChannel, Configuration config) {
        SparkMaxU spark = new SparkMaxU(id, pdpChannel);
        configSpark(spark, config);
        return spark;
    }

    private static boolean configSpark(CANSparkMax spark, Configuration config) {

        boolean setSucceeded;
        int retryCounter = 0;

        do {
            setSucceeded = true;

            setSucceeded &= spark.clearFaults() == REVLibError.kOk;

            spark.setControlFramePeriodMs(config.CONTROL_FRAME_PERIOD_MS);

        } while(!setSucceeded && retryCounter++ < Constants.kSparkMaxRetryCount);

        return setSucceeded;
    }

}
