package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpitterSubsystem extends SubsystemBase {

    //change PID (if needed)
    private static double kP = 0.0;
    private static double kI = 0.0;
    private static double kD = 0.0;

    // change can ID
    private static final int LS_CAN_ID = -10;
    private static TalonFX leftSpitter;
    private static TalonFXConfiguration LSMotorConfig;

    // change can ID
    private static final int RS_CAN_ID = -4;
    private TalonFX rightSpitter;
    private TalonFXConfiguration RSMotorConfig;

    // change can ID
    private static final int LF_CAN_ID = -19;
    private static TalonFX leftFeeder;
    private static TalonFXConfiguration LFMotorConfig;

    // change can ID
    private static final int RF_CAN_ID = 48;
    private TalonFX rightFeeder;
    private TalonFXConfiguration RFMotorConfig;

    // change amp limit
    private static final Current CURRENT_LIMIT = Amps.of(80);

    public SpitterSubsystem() {
        // initializes shooting motors
        leftSpitter = new TalonFX(LS_CAN_ID, "Placeholder"); // placeholder name for the canbus
        LSMotorConfig.Slot0.kP = (kP);
        LSMotorConfig.Slot0.kI = (kI);
        LSMotorConfig.Slot0.kD = (kD);
        LSMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            );
        LSMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftSpitter.getConfigurator().apply(LSMotorConfig);

        rightSpitter = new TalonFX(RS_CAN_ID, "Placeholder"); // placeholder name for the canbus
        RSMotorConfig.Slot0.kP = (kP);
        RSMotorConfig.Slot0.kI = (kI);
        RSMotorConfig.Slot0.kD = (kD);
        RSMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            );
        RSMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightSpitter.getConfigurator().apply(RSMotorConfig);

    
        // initializes feeding motors
        leftFeeder= new TalonFX(LF_CAN_ID, "Placeholder"); // placeholder name for the canbus
        LFMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            );
        LFMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftFeeder.getConfigurator().apply(LFMotorConfig);


        // initializes feeding motors
        rightFeeder = new TalonFX(RF_CAN_ID, "Placeholder"); // placeholder name for the canbus
        RFMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            );
        RFMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightFeeder.getConfigurator().apply(RSMotorConfig);

    }

    public Command spit(double speed) {
        return new Command() {
            @Override
            public void initialize() {
                System.out.println("Spitting");
            }

            @Override
            public void execute() {
                rightSpitter.set(speed);
                leftSpitter.set(speed);
            }

            @Override
            public void end(boolean interrupted) {
                rightSpitter.stopMotor();
                leftSpitter.stopMotor();
            }
        };
    }
}