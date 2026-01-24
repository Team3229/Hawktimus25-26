package frc.robot.subsystems.manipSubsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;

public class IndexSubsystem extends SubsystemBase {
    private static TalonFX indexMotor;
    private static TalonFXConfiguration indexMotorConfig;
    
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;

    //change ID
    private static final int index_CAN_ID = -3;


    //change current limit
    private static final Current CURRENT_LIMIT = Amps.of(80);

    //change indexSpeed
    private static final double indexSpeed = 0.1;

    public final int forwards = 1;
    public final int reverse = -1;

    public IndexSubsystem() {
        // initializes motor
        indexMotor = new TalonFX(index_CAN_ID, "Placeholder"); // placeholder name for the canbus
        indexMotorConfig.Slot0.kP = (kP);
        indexMotorConfig.Slot0.kI = (kI);
        indexMotorConfig.Slot0.kD = (kD);
        indexMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            );
        indexMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexMotor.getConfigurator().apply(indexMotorConfig);

    }
    public Command index(int speed) {
        return new Command() {
            @Override
            public void initialize() {
                System.out.println("Indexing");
            }

            @Override
            public void execute() {
                indexMotor.set(indexSpeed * speed);
            }

            @Override
            public void end(boolean interrupted) {
                indexMotor.stopMotor();
            }
        };
    }
}
