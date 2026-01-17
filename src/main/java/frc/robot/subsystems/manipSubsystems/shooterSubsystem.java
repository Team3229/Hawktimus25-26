package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterSubsystem extends SubsystemBase {
    
    private static final int LS_CAN_ID = 0;
    private SparkMax leftShooter;
    private SparkMaxConfig LSMotorConfig;

    private static final int RS_CAN_ID = -4;
    private SparkMax rightShooter;
    private SparkMaxConfig RSMotorConfig;

    private static final Current CURRENT_LIMIT = Amps.of(80);
    private static final boolean INVERTED = false;

    public void flywheelShooter() {

        // initializes motor
        leftShooter = new SparkMax(LS_CAN_ID, MotorType.kBrushless);
       
       
        LSMotorConfig = new SparkMaxConfig();
        LSMotorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
        
        LSMotorConfig.inverted(!INVERTED);
        LSMotorConfig.idleMode(IdleMode.kBrake);
       
        leftShooter.configure(
            LSMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        rightShooter = new SparkMax(RS_CAN_ID, MotorType.kBrushless);


        RSMotorConfig = new SparkMaxConfig();
        RSMotorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));

        RSMotorConfig.inverted(INVERTED);
        RSMotorConfig.idleMode(IdleMode.kBrake);

        rightShooter.configure(
            RSMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

        public Command shoot(double speed) {
            return new Command() {
                @Override
                public void initialize() {
                    System.out.println("Shooting");
                }

                @Override
                public void execute() {
                    rightShooter.set(speed);
                    leftShooter.set(speed);
                }

                @Override
                public void end(boolean interrupted) {
                    rightShooter.stopMotor();
                    leftShooter.stopMotor();
                }
            };
        }

    public static void shooterArm() {
        //auto angle for the hub

        //manual angle control
    }
}
