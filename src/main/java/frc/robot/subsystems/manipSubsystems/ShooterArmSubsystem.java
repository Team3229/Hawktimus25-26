package frc.robot.subsystems.manipSubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ShooterArmSubsystem extends SubsystemBase {

    private SparkMax armMotor;
    private SparkMaxConfig armMotorConfig;
    private ArmFeedforward feedForward;
    private ProfiledPIDController pidController;

    private static final int CAN_ID = -1; // Change
    private static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;

    // Change angle
    public static final Angle EXAMPLE_ANGLE = Degrees.of(64);

    private static final double kP = 2.5;
    private static final double kI = 0;
    private static final double kD = 0;

    private static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(5);
    private static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(5);

    private static final Angle POSITION_TOLERANCE = Degrees.of(0.1);
    private static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0.1);


    public ShooterArmSubsystem() {

        armMotor = new SparkMax(CAN_ID, MotorType.kBrushless);

        armMotorConfig = new SparkMaxConfig();

        feedForward = new ArmFeedforward(
            0,
            0.83,
            0.39,
            0.02
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

        setSetpoint(HOME_POSITION);
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

    public Angle getPosition() {
        return Radians.of(armMotor.getAbsoluteEncoder().getPosition());
    }

    public AngularVelocity getVelocity() {
        return RadiansPerSecond.of(armMotor.getAbsoluteEncoder().getVelocity());
    }

    private void setSetpoint(Angle setpoint) {
        pidController.setGoal(setpoint.in(Radians));
    }

    private boolean atGoal() {
        return pidController.atGoal();
    }

    public Command rotateTo(Angle setpoint) {
        return runOnce(
            () -> setSetpoint(setpoint)
        ).until(
            () -> atGoal()
        );
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

    // example for later
    public double alignToHub(double example) {
        return example
    }
}