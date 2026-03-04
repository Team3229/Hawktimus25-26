package frc.robot.subsystems.manipSubsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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

    private double requestedSpeed;
    
    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    private final double kV = 0;

    //change ID
    private static final int index_CAN_ID = 0;

    //change current limit
    private static final Current CURRENT_LIMIT = Amps.of(40);

    //change indexSpeed

    public final int forwards = 10;
    public final int reverse = -10;

    public IndexSubsystem() {
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
            );
        indexMotorConfig.Slot0.kP = (kP);
        indexMotorConfig.Slot0.kI = (kI);
        indexMotorConfig.Slot0.kD = (kD);
        indexMotorConfig.Slot0.kV = (kV);
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
                indexMotor.setControl(new VelocityVoltage(rps).withSlot(0));
            }

            @Override
            public void end(boolean interrupted) {
                indexMotor.stopMotor();
            }
        };
        SmartDashboard.putData("Index", new Sendable() {
            @Override 
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Indexing", ()-> isReady(), null);
            }
        });

        out.addRequirements(this);
            return out;
    }
    

    public boolean isReady() {
        double deadBand = 1;
        double indexVelocity = indexMotor.getVelocity().getValueAsDouble();
        if (requestedSpeed == 0) {
            return false;
        } else {
            return Math.abs(requestedSpeed - indexVelocity) <= deadBand;
        }
    }
}
