package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSubsystem extends SubsystemBase {

    private static SparkMax armMotor;
    private static SparkMaxConfig armMotorConfig;
    private static ArmFeedforward feedForward;
    private static ProfiledPIDController pidController;

    private static final int CAN_ID = 9;
    private static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;

    public static final Angle HOME_ANGLE = Degrees.of(345);
    public static final Angle COLLECTION_POINT = Degrees.of(270);

    private intakeSubsystem arm;
    private intakeSubsystem intakeRod1;
    private intakeSubsystem intakeRod2;

    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;

    private static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(0);
    private static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(0);

    private static final Angle POSITION_TOLERANCE = Degrees.of(0);
    private static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0);


    public static void intakeSubsytem() {   

        armMotor = new SparkMax(CAN_ID, MotorType.kBrushless);

        armMotorConfig = new SparkMaxConfig();

        feedForward = new ArmFeedforward(
            0,
            0,
            0,
            0
        );

        pidController = new ProfiledPIDController(
            kP,
            kI,
            kD,
            new Constraints(
                MAX_VELOCITY.in(RadiansPerSecond),
                MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
            )
        );

        pidController.setTolerance(
            POSITION_TOLERANCE.in(Radians),
            VELOCITY_TOLERANCE.in(RadiansPerSecond)
        );

        setSetpoint(HOME_ANGLE);
        pidController.reset(getPosition().in(Radians));

        armMotorConfig.absoluteEncoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(POSITION_CONVERSION_FACTOR)
            .zeroCentered(true);

        armMotor.configure(
            armMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

     public static Angle getPosition() {
        return Radians.of(armMotor.getAbsoluteEncoder().getPosition());
    }

    public static AngularVelocity getVelocity() {
        return RadiansPerSecond.of(armMotor.getAbsoluteEncoder().getVelocity());
    }

    public Command rotateTo(Angle setpoint) {
        return runOnce(
            () -> setSetpoint(setpoint)
        ).until(
            () -> atGoal()
        );
    }

    public Command spin(boolean clockwise) {
        return Commands.runOnce(
            () -> setRodDirection(clockwise)
        );
    }

    private void setWheelDirection(boolean clockwise) {

        if (clockwise) {
            wheelMotor.set(CW_SPEED);
        } else {
            wheelMotor.set(CCW_SPEED);
        }

    }
    public double getDraw() {
        return wheelMotor.getOutputCurrent();
    }

    private static void setSetpoint(Angle setpoint) {
        pidController.setGoal(setpoint.in(Radians));
    }

    private boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public void periodic() {
        armMotor.setVoltage(
            pidController.calculate(getPosition().in(Radians)) +
            feedForward.calculate(
                pidController.getSetpoint().position,
                pidController.getSetpoint().velocity
            )
        );
    }

        //intake/storage push out function
        
        public Command extendIntake() {
            return arm.rotateTo(intakeSubsystem.COLLECTION_POINT);
        }

        //turn intake on function

        public Command intakeCommand() {
            return intakeRod1.spin();
            .alongWith(intakeRod2.spin());
        }


        //roller indexer function (runs while intaking or toggle on/off)

        //extake (run intake and indexer in reverse) for human player station and in panic moments
        
        //intake push function

    
}

