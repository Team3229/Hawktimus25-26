package frc.robot.subsystems.manipSubsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.VoltageConfigs;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.Sendable;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;

import static edu.wpi.first.units.Units.Amps;

public class SpitterSubsystem extends SubsystemBase {
    private static DriveSubsystem driveSubsystem;
    private static double requestedShooterVelocity = 26;
    private static double requestedFeederVelocity = 38;
    private static double deadBand = 2.5;

    // change PID (if needed)
    private static double kP = 0.1;
    private static double kV = 0.13;

    private static double fP = 0.1;
    private static double fV = 0.13;

    private static final int LS_CAN_ID = 10;
    private TalonFX leftSpitter;
    private TalonFXConfiguration LSMotorConfig;

    private static final int RS_CAN_ID = 12;
    private TalonFX rightSpitter;
    private TalonFXConfiguration RSMotorConfig;

    private static final int Feeder_CAN_ID = 11;
    private TalonFX feeder;
    private TalonFXConfiguration feederMotorConfig;

    private static final Current CURRENT_LIMIT = Amps.of(40);

    public record SpitterParams(double srps, double frps, double timeOfFlight) {}

    public static final InterpolatingTreeMap<Double, SpitterParams> SPITTER_MAP = new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(), 
        (start, end, t) -> new SpitterParams(
            MathUtil.interpolate(start.srps, end.srps, t), 
            MathUtil.interpolate(start.frps, end.frps, t), 
            MathUtil.interpolate(start.timeOfFlight, end.timeOfFlight, t)
        )
    );

    static {
        // SPITTER_MAP.put(1.782, new SpitterParams(1, 1, 1));
        SPITTER_MAP.put(2.745, new SpitterParams(28, 40, 0.68));
        SPITTER_MAP.put(3.6576, new SpitterParams(31, 41, 0.9));
        SPITTER_MAP.put(4.44, new SpitterParams(35, 41, 1.2));
        SPITTER_MAP.put(5.33, new SpitterParams(39, 43, 1.34));
    }

    public static final double SYSTEM_LATENCY_SECONDS = 0.3;

    private static Sendable spitterSendable;

    public SpitterSubsystem(DriveSubsystem drive) {
        driveSubsystem = drive;

        // initializes shooting motor
        leftSpitter = new TalonFX(LS_CAN_ID, CANBus.roboRIO()); 
        LSMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12)
                    .withSupplyVoltageTimeConstant(0)
            );

        LSMotorConfig.Slot0.kP = kP;
        LSMotorConfig.Slot0.kV = kV;
        
        LSMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftSpitter.getConfigurator().apply(LSMotorConfig);

        rightSpitter = new TalonFX(RS_CAN_ID, CANBus.roboRIO()); 
        RSMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12)
                    .withSupplyVoltageTimeConstant(0)
            );

        RSMotorConfig.Slot0.kP = kP;
        RSMotorConfig.Slot0.kV = kV;
        RSMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightSpitter.getConfigurator().apply(RSMotorConfig);

        // initializes feeding motor
        feeder = new TalonFX(Feeder_CAN_ID, CANBus.roboRIO()); 
        feederMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12)
                    .withSupplyVoltageTimeConstant(0)
            );
            // .withFeedback(
            //     new FeedbackConfigs()
            //         .withSensorToMechanismRatio(sensorToMechanismRatio)
		    // );
        
        feederMotorConfig.Slot0.kP = fP;
        feederMotorConfig.Slot0.kV = fV;
        feederMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feeder.getConfigurator().apply(feederMotorConfig);

        spitterSendable = new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("LSpitterVelocity", () -> leftSpitter.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("RSpitterVelocity", () -> rightSpitter.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("KickerVelocity", () -> feeder.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("SRPS", () -> requestedShooterVelocity, null);
                builder.addDoubleProperty("FRPS", () -> requestedFeederVelocity, null);
                builder.addBooleanProperty("Ready to Shoot", () -> shooterIsReady(), null);
                builder.addBooleanProperty("Feeder is ready", () -> feederIsReady(), null);
                
            };
        };
		SmartDashboard.putData("Spitter", spitterSendable);
    }

    public Command shoot() {
        Command out = new Command() {
            @Override
            public void execute() {
                setSpitterSpeeds(); // REMOVE FOR MANUAL
                leftSpitter.setControl(new VelocityVoltage(requestedShooterVelocity).withSlot(0).withFeedForward(0.12));
                rightSpitter.setControl(new VelocityVoltage(requestedShooterVelocity).withSlot(0).withFeedForward(0.12));
                feeder.setControl(new VelocityVoltage(requestedFeederVelocity).withSlot(0));
            }

            @Override
            public void end(boolean interrupted) {
                leftSpitter.setControl(new VelocityVoltage(0).withSlot(0));
                rightSpitter.setControl(new VelocityVoltage(0).withSlot(0));
                feeder.setControl(new VelocityVoltage(0).withSlot(0));
            }
        };

        out.addRequirements(this);

        return out;
    }

    public boolean shooterIsReady() {
        double leftVelocity = leftSpitter.getVelocity().getValueAsDouble();
        double rightVelocity = rightSpitter.getVelocity().getValueAsDouble();
        if(requestedShooterVelocity == 0) {
            return false;
        } else {
            return Math.abs(requestedShooterVelocity - leftVelocity) <= deadBand 
            && Math.abs(requestedShooterVelocity - rightVelocity) <= deadBand;
        }
    }
   
    public boolean feederIsReady() {
        double feederVelocity = feeder.getVelocity().getValueAsDouble();
        if(requestedFeederVelocity == 0) {
            return false;
        } else {
            return Math.abs(requestedFeederVelocity - feederVelocity) <= deadBand;
        }
    }

     private void upSRPS() {
        requestedShooterVelocity = Math.min(requestedShooterVelocity + 1, 100);
    }

    public Command upSRPSCommand() {
        return runOnce(() -> {
            upSRPS(); 
            System.out.println("We lowered SRPS: " + requestedShooterVelocity);
        });
    }

    private void downSRPS() {
        requestedShooterVelocity = Math.max(requestedShooterVelocity - 1, 5);
    }

    public Command downSRPSCommand() {
        return runOnce(() -> {
            downSRPS(); 
            System.out.println("WE lowered SRPS to: " + requestedShooterVelocity);
        });
    }

     private void upFRPS() {
        requestedFeederVelocity = Math.min(requestedFeederVelocity + 1, 100);
    }
    
    public Command upFRPSCommand() { 
        return runOnce(() -> {
            upFRPS(); 
            System.out.println("WE raised FRPS to: " + requestedFeederVelocity);
        });
    }

    private void downFRPS() {
        requestedFeederVelocity = Math.max(requestedFeederVelocity - 1, 5);
    }

    public Command downFRPSCommand() {
        return runOnce(() -> {
            downFRPS(); 
            System.out.println("WE lowered FRPS to: " + requestedFeederVelocity);
        });
    }

    public void setFeederSpeed(double distanceMeters) {
		requestedFeederVelocity = SPITTER_MAP.get(distanceMeters).frps();
	}

    public void setShooterSpeed(double distanceMeters) {
        requestedShooterVelocity = SPITTER_MAP.get(distanceMeters).srps();
    }

    public void setSpitterSpeeds() {
        double distanceFromHub = driveSubsystem.distanceToTarget;
        setFeederSpeed(distanceFromHub);   
        setShooterSpeed(distanceFromHub);
    }

}
 