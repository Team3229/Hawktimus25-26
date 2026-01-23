package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.manipSubsystems.IntakeSubsystem;

public class IntakeSubsystem extends SubsystemBase {

    private static SparkMax armMotor;
    private static SparkMax rodMotor;
    private static SparkMaxConfig armMotorConfig;
    private static SparkMaxConfig rodMotorConfig;
    private static ArmFeedforward feedForward;
    private static ProfiledPIDController pidController;

    private static final int ARM_CAN_ID = 9; // TODO: change this
    private static final int ROD_CAN_ID = 8; // TODO: change this
    private static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;

    public static final Angle HOME_ANGLE = Degrees.of(345); // TODO: change this
    public static final Angle COLLECTION_POINT = Degrees.of(270); // TODO: change this

    private static final double kP = 0; // TODO: change this
    private static final double kI = 0; // TODO: change this
    private static final double kD = 0; // TODO: change this

    private static final double CW_SPEED = 0; // TODO: change this
    private static final double CCW_SPEED = 0; // TODO: change this

    private static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(0); // TODO: change this
    private static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(0); // TODO: change this

    private static final Angle POSITION_TOLERANCE = Degrees.of(0); // TODO: change this
    private static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0); // TODO: change this

    public IntakeSubsystem() {

        armMotor = new SparkMax(ARM_CAN_ID, MotorType.kBrushless);

        rodMotor = new SparkMax(ROD_CAN_ID, MotorType.kBrushless);

        armMotorConfig = new SparkMaxConfig();

        rodMotorConfig = new SparkMaxConfig();

        feedForward = new ArmFeedforward(
            0, // TODO: change this
            0, // TODO: change this
            0, // TODO: change this
            0 // TODO: change this
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
            () -> setSetpoint(setpoint)).until(
                () -> atGoal()
            );
    }

    /**
     * creates a command to collect the fuel by spinning the rod
     * 
     * @return Command to spin rod
     */
    public Command intake() {
        return Commands.runOnce(
            () -> rodMotor.set(CW_SPEED) // TODO: Check direction
        );
    }

    /**
     * creates a command to reverse the intake to put the fuel into the human player station 
     * @return Command to spin the rod in reverse
     */
    public Command extake() {
        return Commands.runOnce(
            () -> rodMotor.set(CCW_SPEED) // TODO: Check direction
        );
    }

    /**
     * creates a command that pushes the intake arm down to the collection point
     * and pushes the storage area out.
     * 
     * @return Command to rotate the arm
     */
    public Command extendIntake() {
        return rotateTo(COLLECTION_POINT);
    }
    /**
     * creates a command that pulls the intake arm back to the 
     * home point in order to move the fuel in storage.
     * @return Command to pull arm back
     */
    public Command agitateFuel() {
        return rotateTo(HOME_ANGLE)
        .andThen(new WaitCommand(0.5))
        .andThen(rotateTo(COLLECTION_POINT)); //TODO: Should work but will need a controlled test
    }
    
    public double getDraw() {
        return rodMotor.getOutputCurrent();
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

}
