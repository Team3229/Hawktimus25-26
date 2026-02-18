package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class IntakeSubsystem extends SubsystemBase {

    private static TalonFX arm1Motor;
    private static TalonFX arm2Motor;
    private static TalonFX rodMotor;
    private static CANcoder arm1CanMotor;
    private static CANcoder arm2CanMotor;
    private static TalonFXConfiguration armMotorConfig;
    private static TalonFXConfiguration rodMotorConfig;
    private static CANcoderConfiguration armCanCoderConfig;

    private static ArmFeedforward feedForward;
    private static ProfiledPIDController armPIDController;

    private static final int ARM1_CAN_ID = 9; // TODO: change this
    private static final int ARM2_CAN_ID = 1122; // TODO: change this

    private static final int ROD_CAN_ID = 8; // TODO: change this
    // private static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;

    private static final Current CURRENT_LIMIT = Amps.of(40);

    public static final Angle HOME_ANGLE = Degrees.of(345); // TODO: change this
    public static final Angle COLLECTION_POINT = Degrees.of(270); // TODO: change this

    private static final double aP = 0; // TODO: change this
    private static final double aI = 0; // TODO: change this
    private static final double aD = 0; // TODO: change this

    private static final double rP = 0; // TODO: change this
    private static final double rV = 0; // TODO: change this

    private static final double CW_SPEED = 0; // TODO: change this
    private static final double CCW_SPEED = 0; // TODO: change this

    private static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(0); // TODO: change this
    private static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(0); // TODO: change this

    private static final Angle POSITION_TOLERANCE = Degrees.of(0); // TODO: change this
    private static final AngularVelocity VELOCITY_TOLERANCE = DegreesPerSecond.of(0); // TODO: change this
    
    public IntakeSubsystem() {

        arm1CanMotor = new CANcoder(ARM1_CAN_ID);

        arm1Motor = new TalonFX(ARM1_CAN_ID, CANBus.roboRIO());

        arm2Motor = new TalonFX(ARM2_CAN_ID, CANBus.roboRIO());

        rodMotor = new TalonFX(ROD_CAN_ID, CANBus.roboRIO());

        
        armCanCoderConfig = new CANcoderConfiguration();
        
        rodMotorConfig = new TalonFXConfiguration();
        
        armCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0; // TODO: Change
        armCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        armCanCoderConfig.MagnetSensor.MagnetOffset = 0.0;
        arm1CanMotor.getConfigurator().apply(armCanCoderConfig);
        
        arm2CanMotor.getConfigurator().apply(armCanCoderConfig);        
        
        armMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(
                new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            );
        armMotorConfig.Feedback.FeedbackRemoteSensorID = arm1CanMotor.getDeviceID();
        armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        armMotorConfig.Feedback.SensorToMechanismRatio = 0.0; // TODO: Change
        armMotorConfig.Feedback.RotorToSensorRatio = 0.0; // TODO: Change

        arm1Motor.getConfigurator().apply(armMotorConfig);
        arm2Motor.getConfigurator().apply(armMotorConfig);
        arm2Motor.setControl(new Follower(ARM1_CAN_ID, MotorAlignmentValue.Opposed));

        feedForward = new ArmFeedforward(
                0, // TODO: change this
                0, // TODO: change this
                0, // TODO: change this
                0 // TODO: change this
        );

        rodMotorConfig.Slot0.kP = (rP);
        rodMotorConfig.Slot0.kV = (rV);

        armPIDController = new ProfiledPIDController(
            aP,
            aI,
            aD,
            new Constraints(
                MAX_VELOCITY.in(DegreesPerSecond),
                MAX_ACCELERATION.in(DegreesPerSecondPerSecond)
            )
        );

        armPIDController.setTolerance(
            POSITION_TOLERANCE.in(Degrees),
            VELOCITY_TOLERANCE.in(DegreesPerSecond));
        setSetpoint(HOME_ANGLE);
        armPIDController.reset(getPosition().in(Degrees));
    }

    /**
     * Gets the angle in degrees of the arm from the CAN
     * 
     * @return The degree of the arm
     */
    public static Angle getPosition() {
        return Degrees.of(arm1CanMotor.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Gets the angular velocity of the arm from the CAN
     * 
     * @return The velocity of the arm
     */
    public static AngularVelocity getVelocity() {
        return DegreesPerSecond.of(arm1CanMotor.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Gets the angular acceleration of the arm from the CAN
     * 
     * @return The acceleration of the arm
     */
    public static AngularAcceleration getAcceleration() {
        return DegreesPerSecondPerSecond.of(arm1CanMotor.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Command that rotates the arm to a setpoint
     * 
     * @return Command to rotate arm
     */
    public Command rotateTo(Angle setpoint) {
        return runOnce(
            () -> setSetpoint(setpoint))
        .until(
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
     * creates a command to reverse the intake to put the fuel into the human player
     * station
     * 
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
     * 
     * @return Command to pull arm back
     */
    public Command agitateFuel() {
        return rotateTo(HOME_ANGLE)
        .andThen(new WaitCommand(0.5))
        .andThen(rotateTo(COLLECTION_POINT)); // TODO: Should work but will need a controlled test
    }

    /** gets your currrent current */
    public StatusSignal<Current> getDraw() {
        return rodMotor.getMotorStallCurrent();
    }

    /** sets a setpoint for the arm to go to */
    private static void setSetpoint(Angle setpoint) {
        armPIDController.setGoal(setpoint.in(Degrees));
    }

    private boolean atGoal() {
        return armPIDController.atGoal();
    }

    /**
     * This command runs 3 values from the motor (angle, velocity, acceleration), and uses them 
     * to calculate a feed forward loop
     */

    @Override
    public void periodic() {
        double currentAngle = arm1Motor.getPosition().getValueAsDouble();
        double currentVelocity = arm1Motor.getVelocity().getValueAsDouble();
        double nextVelocity = (arm1Motor.getAcceleration().getValueAsDouble() + currentVelocity) / 10;

        arm1Motor.setVoltage(
            armPIDController.calculate(getPosition().in(Degrees)) +
            feedForward.calculateWithVelocities(
                currentAngle,
                currentVelocity,
                nextVelocity
            )
        );
    }
}