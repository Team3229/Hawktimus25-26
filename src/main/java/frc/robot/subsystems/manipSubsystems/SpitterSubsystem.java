package frc.robot.subsystems.manipSubsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.VoltageConfigs;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.Sendable;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;

import static edu.wpi.first.units.Units.Amps;

import java.util.Date;

public class SpitterSubsystem extends SubsystemBase {
    private static DriveSubsystem driveSubsystem;
    private static double requestedShooterVelocity = 28;
    private static double requestedFeederVelocity = 13;
    private static double deadBand = 1;

    // change PID (if needed)
    private double kP = 0.2;
    private double kV = 0.2027;
    private double kS = 0.2111;

    private double fP = 0.25;
    private double fV = 0.71;
    private double fS = 0.012;

    private static final int LS_CAN_ID = 10; 
    private TalonFX leftSpitter;
    private TalonFXConfiguration shooterMotorConfig;

    private static final int RS_CAN_ID = 12;
    private TalonFX rightSpitter;

    private static final int Feeder_CAN_ID = 11;
    private TalonFX feeder;
    private TalonFXConfiguration feederMotorConfig;

    private long spitTimer = 0;

    private double feederSensorToMechanismRatio = 6;
    private double shooterSensorToMechanismRatio = 1.75;

    private static final Current CURRENT_LIMIT = Amps.of(40);

    private boolean testMode = false; // TODO: 

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
        // SPITTER_MAP.put(1.97, new SpitterParams(40, 14, 1));
        SPITTER_MAP.put(2.42, new SpitterParams(42, 13, 1));
        SPITTER_MAP.put(2.93, new SpitterParams(44, 13, 0.68));
        SPITTER_MAP.put(3.42, new SpitterParams(50, 14, 1.2));
        SPITTER_MAP.put(3.57, new SpitterParams(51, 14, 1.2));
        SPITTER_MAP.put(3.83, new SpitterParams(55, 14, 0.9));

    }

    public static final double SYSTEM_LATENCY_SECONDS = 0.3;

    private static Sendable spitterSendable;
    private static Sendable spitterPIDSendable;
    private static Sendable feederPIDSendable;


    public SpitterSubsystem(DriveSubsystem drive) {
        driveSubsystem = drive;

        // initializes shooting motor
        leftSpitter = new TalonFX(LS_CAN_ID, CANBus.roboRIO()); 
        shooterMotorConfig = new TalonFXConfiguration()
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
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(shooterSensorToMechanismRatio)
            );

        shooterMotorConfig.Slot0.withKP(kP);
        shooterMotorConfig.Slot0.withKV(kV);
        shooterMotorConfig.Slot0.withKS(kS);
        
        shooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftSpitter.getConfigurator().apply(shooterMotorConfig);

        rightSpitter = new TalonFX(RS_CAN_ID, CANBus.roboRIO()); 
        rightSpitter.getConfigurator().apply(shooterMotorConfig);
        rightSpitter.setControl(new Follower(LS_CAN_ID, MotorAlignmentValue.Opposed));        

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
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(feederSensorToMechanismRatio)
		    );
        
        feederMotorConfig.Slot0.withKP(fP);
        feederMotorConfig.Slot0.withKV(fV);
        feederMotorConfig.Slot0.withKS(fS);
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
                builder.addIntegerProperty("SpitTime", () -> spitTimer, null);
                builder.addDoubleProperty("Spitter accel", () -> requestedShooterVelocity/1.5, null);
                builder.addDoubleProperty("Left acceleration", () -> leftSpitter.getAcceleration().getValueAsDouble(), null);
                builder.addDoubleProperty("Right acceleration", () -> rightSpitter.getAcceleration().getValueAsDouble(), null);
            };
        };
        SmartDashboard.putData("Spitter", spitterSendable);

        spitterPIDSendable = new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Spitter P", () -> kP, (newkP) -> editSpitterP(newkP));
                builder.addDoubleProperty("Spitter V", () -> kV, (newkV) -> editSpitterV(newkV));
                builder.addDoubleProperty("Spitter S", () -> kS, (newkS) -> editSpitterS(newkS));
            }
        };

        feederPIDSendable = new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Feeder P", () -> fP, (newfP) -> editFeederP(newfP));
                builder.addDoubleProperty("Feeder V", () -> fV, (newfV) -> editFeederV(newfV));
                builder.addDoubleProperty("Feeder S", () -> fS, (newfS) -> editFeederS(newfS));
            }
        };

        if(testMode) {
            SmartDashboard.putData("ShooterPID", spitterPIDSendable);
            SmartDashboard.putData("FeederPID", feederPIDSendable);
        }
    }

    //TODO: delete
    public Command spinShooter() {
        Command out = new Command() {
            Date start;
            Date end;
            @Override
            public void initialize() {
                start = new Date();
                end = null;
            }
            @Override
            public void execute() {
                leftSpitter.setControl(new VelocityVoltage(requestedShooterVelocity).withSlot(0));
                if (shooterIsReady() && end == null) {
                    end = new Date();
                    spitTimer = end.getTime() - start.getTime();
                    System.out.println("got to speed in: " + spitTimer + " milliseconds");
                }
            }

            @Override
            public void end(boolean interrupted) {
                leftSpitter.setControl(new CoastOut());
            }
        };

        out.addRequirements(this);

        return out;
    }

    public Command spinKicker() {
        Command out = new Command() {
            @Override
            public void execute() {
                // setSpitterSpeeds(); // REMOVE FOR MANUAL
                feeder.setControl(new VelocityVoltage(requestedFeederVelocity).withSlot(0));
            }

            @Override
            public void end(boolean interrupted) {
                feeder.setControl(new CoastOut());
            }
        };

        out.addRequirements(this);

        return out;
    }
    //TODO: delete

    public Command shoot() {
        Command out = new Command() {
            Date start;
            Date end;
            @Override
            public void initialize() {
                start = new Date();
                end = null;
            }
            @Override
            public void execute() {
                setShooterSpeed(); // REMOVE FOR MANUAL
                leftSpitter.setControl(new VelocityVoltage(requestedShooterVelocity).withSlot(0));
                feeder.setControl(new VelocityVoltage(requestedFeederVelocity).withSlot(0));
                if (shooterIsReady() && end == null) {
                    end = new Date();
                    spitTimer = end.getTime() - start.getTime();
                    System.out.println("got to speed in: " + spitTimer + " milliseconds");
                }
            }

            @Override
            public void end(boolean interrupted) {
                leftSpitter.setControl(new CoastOut());
                feeder.setControl(new CoastOut());
            }
        };

        out.addRequirements(this);

        return out;
    }

    private void editSpitterP(double newkP) {
        kP = newkP;
        shooterMotorConfig.Slot0.kP = kP;

        leftSpitter.getConfigurator().apply(shooterMotorConfig);
        rightSpitter.getConfigurator().apply(shooterMotorConfig);

    }

    private void editSpitterV(double newkV) {
        kV = newkV;
        shooterMotorConfig.Slot0.kV = kV;

        leftSpitter.getConfigurator().apply(shooterMotorConfig);
        rightSpitter.getConfigurator().apply(shooterMotorConfig);
    }

    private void editSpitterS(double newkS) {
        kS = newkS;
        shooterMotorConfig.Slot0.kS = kS;

        leftSpitter.getConfigurator().apply(shooterMotorConfig);
        rightSpitter.getConfigurator().apply(shooterMotorConfig);
    }

    private void editFeederP(double newfP) {
        fP = newfP;
        feederMotorConfig.Slot0.kP = fP;

        feeder.getConfigurator().apply(feederMotorConfig);
    }

    private void editFeederV(double newfV) {
        System.out.println("old fV: " + fV);
        fV = newfV;
        feederMotorConfig.Slot0.kV = fV;

        feeder.getConfigurator().apply(feederMotorConfig);
        System.out.println("new fV: " + feederMotorConfig.Slot0.kV);
    }

    private void editFeederS(double newfS) {
        fS = newfS;
        feederMotorConfig.Slot0.kV = fS;

        feeder.getConfigurator().apply(feederMotorConfig);
    }
    
    public boolean shooterIsReady() {
        double leftVelocity = leftSpitter.getVelocity().getValueAsDouble();
        double rightVelocity = rightSpitter.getVelocity().getValueAsDouble();
        if(requestedShooterVelocity == 0) {
            return false;
        } else {
            return (Math.abs(requestedShooterVelocity - leftVelocity) + Math.abs(requestedShooterVelocity - rightVelocity))/2 <= deadBand;
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

    public Command upSRPSCommand() {
        return runOnce(() -> {
            requestedShooterVelocity = Math.min(requestedShooterVelocity + 1, 100); 
            System.out.println("SRPS raised to: " + requestedShooterVelocity);
        });
    }

    public Command downSRPSCommand() {
        return runOnce(() -> {
            requestedShooterVelocity = Math.max(requestedShooterVelocity - 1, 5);
            System.out.println("SRPS lowered to: " + requestedShooterVelocity);
        });
    }
    
    public Command upFRPSCommand() { 
        return runOnce(() -> {
            requestedFeederVelocity = Math.min(requestedFeederVelocity + 1, 100);
            System.out.println("FRPS raised to: " + requestedFeederVelocity);
        });
    }

    public Command downFRPSCommand() {
        return runOnce(() -> {
            requestedFeederVelocity = Math.max(requestedFeederVelocity - 1, 5);
            System.out.println("FRPS lowered to: " + requestedFeederVelocity);
        });
    }

    public void setShooterSpeed() {
        double distanceFromHub = driveSubsystem.distanceToTarget;
        requestedShooterVelocity = SPITTER_MAP.get(distanceFromHub).srps();
    }

}