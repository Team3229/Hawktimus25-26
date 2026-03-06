package frc.robot.subsystems.manipSubsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Current;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;

public class IndexSubsystem extends SubsystemBase {
    private static TalonFX indexMotor;
    private static TalonFXConfiguration indexMotorConfig;

    private SpitterSubsystem spitterSubsystem;

    private double requestedSpeed;

    private double sensorToMechanismRatio = 5;
    
    private static double kP = 0.5;
    private static double kV = 0.13;

    //change ID
    private static final int index_CAN_ID = 6;

    //change current limit
    private static final Current CURRENT_LIMIT = Amps.of(40);

    //change indexSpeed
    
    // variables are halved when ran for some reason :(
    public final int forwards = 30;
    public final int reverse = -20;

    public IndexSubsystem(SpitterSubsystem spit) {
        spitterSubsystem = spit;
        // initializes motor
        indexMotor = new TalonFX(index_CAN_ID, CANBus.roboRIO()); // placeholder name for the canbus
        indexMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            )
            .withFeedback(
			    new FeedbackConfigs()
			        .withSensorToMechanismRatio(sensorToMechanismRatio)
		    );
        indexMotorConfig.Slot0.kP = kP;
        indexMotorConfig.Slot0.kV = kV;
        indexMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        indexMotor.getConfigurator().apply(indexMotorConfig);
    }
    
    /**Command to run the index motors */
    public Command index(double rps) {
        Command out = new Command() {
            @Override
            public void initialize() {
                System.out.println("Indexing");
                requestedSpeed = rps;
            }

            @Override
            public void execute() {
                if(rps != forwards || spitterSubsystem.shooterIsReady()) {
                    indexMotor.setControl(new VelocityVoltage(rps).withSlot(0));
                }
            }

            @Override
            public void end(boolean interrupted) {
                indexMotor.setControl(new VelocityVoltage(0).withSlot(0));
            }
        };
        SmartDashboard.putData("Index", new Sendable() {
            @Override 
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Indexing", ()-> isReady(), null);
                builder.addDoubleProperty("IndexActual", () -> indexMotor.getVelocity().getValueAsDouble(), null);

            }
        });

        out.addRequirements(this);
        return out;
    }
    

    public boolean isReady() {
        double deadBand = 0.1;
        double indexVelocity = indexMotor.getVelocity().getValueAsDouble();
        if (requestedSpeed == 0) {
            return false;
        } else {
            return Math.abs(requestedSpeed - indexVelocity) <= deadBand;
        }
    }

}
