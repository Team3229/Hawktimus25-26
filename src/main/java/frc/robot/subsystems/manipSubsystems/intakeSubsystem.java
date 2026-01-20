package frc.robot.subsystems.manipSubsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Current;
import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSubsystem extends SubsystemBase {

    public static void intakeSubsytem() {
        // intake/storage push out function
        /*
         * if we have a motor make it spin
         * 
         * if we don't the intake pushes it
         */

        // turn intake on function

        // roller indexer function (runs while intaking or toggle on/off)

        // extake (run intake and indexer in reverse) for human player station and in
        // panic moments

        // intake push function

    }

    private static final int index_CAN_ID = -3;
    private SparkMax indexMotor;
    private SparkMaxConfig indexMotorConfig;

    private static final Current CURRENT_LIMIT = Amps.of(80);
    private static final boolean INVERTED = true;

    private static final double indexSpeed = 0.1;

    public void indexSubsystem() {
        // initializes motor
        indexMotor = new SparkMax(index_CAN_ID, MotorType.kBrushless);

        indexMotorConfig = new SparkMaxConfig();

        indexMotorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
        indexMotorConfig.inverted(!INVERTED);
        indexMotorConfig.idleMode(IdleMode.kBrake);

        indexMotor.configure(
            indexMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

    public Command index() {
        return new Command() {
            @Override
            public void initialize() {
                System.out.println("Indexing");
            }

            @Override
            public void execute() {
                indexMotor.set(indexSpeed);
            }

            @Override
            public void end(boolean interrupted) {
                indexMotor.stopMotor();
            }
        };
    }
}
