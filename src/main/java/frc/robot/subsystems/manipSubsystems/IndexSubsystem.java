package frc.robot.subsystems.manipSubsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.Amps;

public class IndexSubsystem extends SubsystemBase {
    //change ID
    private static final int index_CAN_ID = -3;
    private static SparkMax indexMotor;
    private static SparkMaxConfig indexMotorConfig;

    //change current limit
    private static final Current CURRENT_LIMIT = Amps.of(80);
    private static final boolean INVERTED = true;

    //change indexSpeed
    private static final double indexSpeed = 0.1;

    public final int forwards = 1;
    public final int reverse = -1;

    //elasitc variable
   public static boolean indexing = false;
    
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
                indexing = true;
            }

            @Override
            public void end(boolean interrupted) {
                indexMotor.stopMotor();
                indexing = false;
            }
        };
    }
}
