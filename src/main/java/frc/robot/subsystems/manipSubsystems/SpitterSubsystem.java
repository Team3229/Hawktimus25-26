package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpitterSubsystem extends SubsystemBase {

    private double requestedVelocity = 0;

    // change PID (if needed)
    private static double kP = 0.1;
    private static double kI = 0;
    private static double kD = 0;
    private static double kV = 0.13;
    // private static double kS = 0;

    private static final double fP = 0.1;
    private static final double fI = 0;
    private static final double fD = 0;
    private static final double fV = 0.12;

    private static final int LS_CAN_ID = 10;
    private TalonFX leftSpitter;
    private TalonFXConfiguration LSMotorConfig;

    private static final int RS_CAN_ID = 8;
    private TalonFX rightSpitter;
    private TalonFXConfiguration RSMotorConfig;

    private static final int Feeder_CAN_ID = 7;
    private TalonFX feeder;
    private TalonFXConfiguration feederMotorConfig;

    // private static final int RF_CAN_ID = 51;
    // private TalonFX rightFeeder;
    // private TalonFXConfiguration RFMotorConfig;

    // change amp limit
    private static final Current CURRENT_LIMIT = Amps.of(40);

    public SpitterSubsystem() {
        // initializes shooting motors
        leftSpitter = new TalonFX(LS_CAN_ID, CANBus.roboRIO()); // placeholder name for the canbus
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

        LSMotorConfig.Slot0.kP = (kP);
        LSMotorConfig.Slot0.kI = (kI);
        LSMotorConfig.Slot0.kD = (kD);
        LSMotorConfig.Slot0.kV = (kV);
        
        // LSMotorConfig.Slot0.kS = (kS);
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

        RSMotorConfig.Slot0.kP = (kP);
        RSMotorConfig.Slot0.kI = (kI);
        RSMotorConfig.Slot0.kD = (kD);
        RSMotorConfig.Slot0.kV = (kV);
        // RSMotorConfig.Slot0.kS = (kS);
        RSMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightSpitter.getConfigurator().apply(RSMotorConfig);
        // rightSpitter.setControl(new StrictFollower(LS_CAN_ID));



        // initializes feeding motors
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
        feederMotorConfig.Slot0.kP = (fP);
        feederMotorConfig.Slot0.kI = (fI);
        feederMotorConfig.Slot0.kD = (fD);
        feederMotorConfig.Slot0.kV = (fV);
        feederMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feeder.getConfigurator().apply(feederMotorConfig);
        // feeder.setControl(new StrictFollower(LS_CAN_ID));


        // rightFeeder = new TalonFX(RF_CAN_ID, "rio"); 
        // RFMotorConfig = new TalonFXConfiguration()
        // .withMotorOutput(
        // new MotorOutputConfigs()
        // .withNeutralMode(NeutralModeValue.Coast)
        // )
        // .withCurrentLimits(
        // new CurrentLimitsConfigs()
        // .withStatorCurrentLimit(CURRENT_LIMIT)
        // .withStatorCurrentLimitEnable(true)
        // );
        // RFMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // RFMotorConfig.Slot0.kP = (kP);
        // RFMotorConfig.Slot0.kI = (kI);
        // RFMotorConfig.Slot0.kD = (kD);
        // rightFeeder.getConfigurator().apply(RSMotorConfig);

    }

    public Command spinUp(double rps) {
        Command out = new Command() {
            @Override
            public void initialize() { 

                System.out.println("raaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaahhhhhhh");
                requestedVelocity = rps;
            }

            // is in execute bc we need to call it every few seconds
            @Override
            public void execute() {
                leftSpitter.setControl(new VelocityVoltage(rps).withSlot(0));
                rightSpitter.setControl(new VelocityVoltage(rps).withSlot(0));
            }

        };
        SmartDashboard.putData("Shooter", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {

                builder.addDoubleProperty("LSpitterVelocity", () -> leftSpitter.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("RSpitterVelocity", () -> rightSpitter.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("KickerVelocity", () -> feeder.getVelocity().getValueAsDouble(), null);
                builder.addBooleanProperty("Ready to Shoot", () -> shooterIsReady(), null);
                builder.addBooleanProperty("Is a Fed", () -> feederIsReady(), null);
                
            }
        });
        out.addRequirements(this);

        return out;
    }

    public Command shoot(double rps) {
        Command out = new Command() {
            @Override
            public void initialize() {
                requestedVelocity = rps;
                System.out.println("LITTLE WHIMSY IS REQUIRED");
            }
            
            @Override
            public void execute() {
                leftSpitter.setControl(new VelocityVoltage(rps).withSlot(0));
                rightSpitter.setControl(new VelocityVoltage(rps).withSlot(0));
                if(shooterIsReady()) {
                    feeder.setControl(new VelocityVoltage(25).withSlot(0));
                } else {
                    feeder.setControl(new VelocityVoltage(0).withSlot(0));
                }
            }
            
        };

        out.addRequirements(this);

        return out;
    }

    public boolean shooterIsReady() {
        double deadBand = 2;
        double leftVelocity = leftSpitter.getVelocity().getValueAsDouble();
        double rightVelocity = rightSpitter.getVelocity().getValueAsDouble();
        if(requestedVelocity == 0) {
            return false;
        } else {
            return Math.abs(requestedVelocity - leftVelocity) <= deadBand 
            && Math.abs(requestedVelocity - rightVelocity) <= deadBand;
        }
    }
   
        public boolean feederIsReady() {
        double deadBand = 2;
        double feederVelocity = feeder.getVelocity().getValueAsDouble();
        if(requestedVelocity == 0) {
            return false;
        } else {
            return Math.abs(requestedVelocity - feederVelocity) <= deadBand;
        }
    }

}