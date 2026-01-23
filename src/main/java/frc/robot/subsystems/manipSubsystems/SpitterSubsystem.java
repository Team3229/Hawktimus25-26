package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpitterSubsystem extends SubsystemBase {

    //change can ID
    private static final int LS_CAN_ID = 0;
    private static SparkMax leftSpitter;
    private static SparkMaxConfig LSMotorConfig;

    //change can ID
    private static final int RS_CAN_ID = -4;
    private SparkMax rightSpitter;
    private SparkMaxConfig RSMotorConfig;

    //change amp limit
    private static final Current CURRENT_LIMIT = Amps.of(80);
    private static final boolean INVERTED = true;

    public static boolean Shooter_Ready = false;

    

    public SpitterSubsystem() {
        // initializes motors
        leftSpitter = new SparkMax(LS_CAN_ID, MotorType.kBrushless);

        LSMotorConfig = new SparkMaxConfig();

        SmartDashboard.putBoolean("Shooter Ready", );
        SmartDashboard.putBoolean("Shooting", indexing);

        LSMotorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
        LSMotorConfig.inverted(!INVERTED);
        LSMotorConfig.idleMode(IdleMode.kBrake);

        leftSpitter.configure(
                LSMotorConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);

        rightSpitter = new SparkMax(RS_CAN_ID, MotorType.kBrushless);

        RSMotorConfig = new SparkMaxConfig();
        RSMotorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));

        RSMotorConfig.inverted(INVERTED);
        RSMotorConfig.idleMode(IdleMode.kBrake);

        rightSpitter.configure(
                RSMotorConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public Command spit(double speed) {
        return new Command() {
            @Override
            public void initialize() {
                System.out.println("Spitting");
            }

            @Override
            public void execute() {
                rightSpitter.set(speed);
                leftSpitter.set(speed);

                if(Math.abs(speed - leftSpitter.get()) <= SPEED_TOLERANCE) {
                    Shooter_Ready = true;
                } else{
                    Shooter_Ready = false;
                }

            }

            @Override
            public void end(boolean interrupted) {
                rightSpitter.stopMotor();
                leftSpitter.stopMotor();
            }
        };
    }
}
