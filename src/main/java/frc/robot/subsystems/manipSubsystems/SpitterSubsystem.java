package frc.robot.subsystems.manipSubsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.VoltageConfigs;

import com.ctre.phoenix6.CANBus;
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
    private static double requestedShooterVelocity = 25;
    private static double requestedFeederVelocity = 35;

    // change PID (if needed)
    private static double kP = 0.1;
    private static double kI = 0;
    private static double kD = 0;
    private static double kV = 0.13;

    private static double fP = 0.3;
    private static double fI = 0;
    private static double fD = 0;
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

    // change amp limit
    private static final Current CURRENT_LIMIT = Amps.of(40);

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
        LSMotorConfig.Slot0.kI = kI;
        LSMotorConfig.Slot0.kD = kD;
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
        RSMotorConfig.Slot0.kI = kI;
        RSMotorConfig.Slot0.kD = kD;
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
        
        feederMotorConfig.Slot0.kP = fP;
        feederMotorConfig.Slot0.kI = fI;
        feederMotorConfig.Slot0.kD = fD;
        feederMotorConfig.Slot0.kV = fV;
        feederMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feeder.getConfigurator().apply(feederMotorConfig);
    }

    public Command shoot() {
        Command out = new Command() {
            @Override
            public void initialize() {

            }
            
            @Override
            public void execute() {
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

     

        SmartDashboard.putData("Shooter", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("LSpitterVelocity", () -> leftSpitter.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("RSpitterVelocity", () -> rightSpitter.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("KickerVelocity", () -> feeder.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("SRPS", () -> requestedShooterVelocity, null);
                builder.addDoubleProperty("FRPS", () -> requestedFeederVelocity, null);
                builder.addBooleanProperty("Ready to Shoot", () -> shooterIsReady(), null);
                builder.addBooleanProperty("Is a Fed", () -> feederIsReady(), null);
                
            }
        });
        out.addRequirements(this);

        return out;
    }

    public Command lowPass() {
        return shoot(30, 75);
    }

    public Command midPass() {
        return shoot(35, 80);
    }

    public Command highPass() {
         return shoot(40, 85);
    }

    public Command shoot(double srps, double frps) {
        Command out = new Command() {
            @Override
            public void initialize() {
                requestedShooterVelocity = srps;
                requestedFeederVelocity = frps;
            }
            
            @Override
            public void execute() {
                leftSpitter.setControl(new VelocityVoltage(srps).withSlot(0));
                rightSpitter.setControl(new VelocityVoltage(srps).withSlot(0));
                feeder.setControl(new VelocityVoltage(frps).withSlot(0));
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
        double deadBand = 2;
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
        double deadBand = 2;
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
            System.out.println("WE UPPED SRPS TO: " + requestedShooterVelocity);
        });
    }

    private void downSRPS() {
        requestedShooterVelocity = Math.max(requestedShooterVelocity - 1, 5);
    }

    public Command downSRPSCommand() {
        return runOnce(() -> {
            downSRPS(); 
            System.out.println("WE lowered SRSPS to:" + requestedShooterVelocity);
        });
    }

     private void upFRPS() {
        requestedFeederVelocity = Math.min(requestedFeederVelocity + 1, 100);
    }
    
    public Command upFRPSCommand() { 
        return runOnce(() -> {
            upFRPS(); 
            System.out.println("WE raised FRSPS to:" + requestedFeederVelocity);
        });
    }

    private void downFRPS() {
        requestedFeederVelocity = Math.max(requestedFeederVelocity - 1, 5);
    }

    public Command downFRPSCommand() {
        return runOnce(() -> {
            downFRPS(); 
            System.out.println("WE lowered FRSPS to:" + requestedFeederVelocity);
        });
    }

    public void setFeederSpeed() {
        requestedFeederVelocity = 0.5 * driveSubsystem.distanceFromHub() + 75;
    }

    public void setShooterSpeed() {
        requestedShooterVelocity = 1.5 * driveSubsystem.distanceFromHub() + 18;
    }

    public Command setSpitterSpeed() {
        return runOnce(() -> {
            setFeederSpeed();   
            setShooterSpeed();
            System.out.println("spitter speed has been set to " + requestedShooterVelocity + " feeder speed has been set to " + requestedFeederVelocity);
        });
    }

}
 