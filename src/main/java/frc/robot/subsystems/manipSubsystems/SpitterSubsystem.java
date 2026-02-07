package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;

public class SpitterSubsystem extends SubsystemBase {

    private double currentSetpoint = 0;
    private static double requestedVelocity = 1;

    //change PID (if needed)
    private static double kP = 1;
    private static double kI = 0;
    private static double kD = 0.1;
    //add F on 2/7/2026


    // change can ID
    private static final int LS_CAN_ID = 10;
    private static TalonFX leftSpitter;
    private static TalonFXConfiguration LSMotorConfig;

    // change can ID
    private static final int RS_CAN_ID = -4;
    private TalonFX rightSpitter;
    private TalonFXConfiguration RSMotorConfig;

    // change can ID
    private static final int LF_CAN_ID = 11;
    private static TalonFX leftFeeder;
    private static TalonFXConfiguration LFMotorConfig;

    // change can ID
    private static final int RF_CAN_ID = 51;
    private TalonFX rightFeeder;
    private TalonFXConfiguration RFMotorConfig;

    // change amp limit
    private static final Current CURRENT_LIMIT = Amps.of(40);

    public SpitterSubsystem() {
        // initializes shooting motors
        leftSpitter = new TalonFX(LS_CAN_ID, "rio"); // placeholder name for the canbus
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
        LSMotorConfig.Slot0.kP = (kP);
        LSMotorConfig.Slot0.kI = (kI);
        LSMotorConfig.Slot0.kD = (kD);
        LSMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftSpitter.getConfigurator().apply(LSMotorConfig);

        // rightSpitter = new TalonFX(RS_CAN_ID, "Placeholder"); // placeholder name for the canbus
        // RSMotorConfig = new TalonFXConfiguration()
        //     .withMotorOutput(
        //         new MotorOutputConfigs()
        //             .withNeutralMode(NeutralModeValue.Brake)
        //     )
        //     .withCurrentLimits(
        //         new CurrentLimitsConfigs()
        //             .withStatorCurrentLimit(CURRENT_LIMIT)
        //             .withStatorCurrentLimitEnable(true)
        //     );
        // RSMotorConfig.Slot0.kP = (kP);
        // RSMotorConfig.Slot0.kI = (kI);
        // RSMotorConfig.Slot0.kD = (kD);
        // RSMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // rightSpitter.getConfigurator().apply(RSMotorConfig);

    
        // initializes feeding motors
        leftFeeder= new TalonFX(LF_CAN_ID, "rio"); // placeholder name for the canbus
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
        LFMotorConfig.Slot0.kP = (kP);
        LFMotorConfig.Slot0.kI = (kI);
        LFMotorConfig.Slot0.kD = (kD);
        LFMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftFeeder.getConfigurator().apply(LFMotorConfig);

        // rightFeeder = new TalonFX(RF_CAN_ID, "rio"); // placeholder name for the canbus
        // RFMotorConfig = new TalonFXConfiguration()
        //     .withMotorOutput(
        //         new MotorOutputConfigs()
        //                 .withNeutralMode(NeutralModeValue.Brake)
        //     )
        //     .withCurrentLimits(
        //         new CurrentLimitsConfigs()
        //             .withStatorCurrentLimit(CURRENT_LIMIT)
        //             .withStatorCurrentLimitEnable(true)
        //     );
        // RFMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // RFMotorConfig.Slot0.kP = (kP);
        // RFMotorConfig.Slot0.kI = (kI);
        // RFMotorConfig.Slot0.kD = (kD);
        // rightFeeder.getConfigurator().apply(RSMotorConfig);

    }

    public Command spinUp(double speed) {
        Command out = new Command() {
            @Override public void initialize() {
                leftSpitter.set(speed);
                // rightSpitter.set(speed);
            }
            @Override public boolean isFinished() {
                return isReady();
            }
        };

        out.addRequirements(this);

        return out;
    }

    public Command shoot(double speed) {
        Command out = new Command() {
            @Override public void initialize() {
                leftFeeder.set(speed);
                // rightFeeder.set(speed);

            }
            @Override public boolean isFinished() {
                return isReady();
            }
        };

        out.addRequirements(this);

        return out;
    }   



    public Command feed() {
        return new Command() {
            @Override
            public void initialize() {
                System.out.println("Feeding");
            }

            @Override
            public void execute() {
                // rightFeeder.set(1);
                leftFeeder.set(0.1);
            }

            @Override
            public void end(boolean interrupted) {
                // rightFeeder.stopMotor();
                leftFeeder.stopMotor();
            }
        };
    }

    public boolean isReady() {
        double deadBand = 0.25;
        double leftVelocity = leftSpitter.getVelocity().getValueAsDouble();
        // double rightVelocity = rightSpitter.getVelocity().getValueAsDouble();

        return Math.abs(requestedVelocity - leftVelocity) <= deadBand;// && Math.abs(requestedVelocity - rightVelocity) <= deadBand;
    }


}