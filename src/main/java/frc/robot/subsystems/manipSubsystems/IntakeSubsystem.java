package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class IntakeSubsystem extends SubsystemBase {

    private static TalonFX armMotor;
    private static TalonFX rodMotor;
    private static CANcoder armCanMotor;
    private static TalonFXConfiguration armMotorConfig;
    private static TalonFXConfiguration rodMotorConfig;
    private static CANcoderConfiguration armCanCoderConfig;


    private static ArmFeedforward feedForward;
    private static ProfiledPIDController armPIDController;

    private static final int ARM_CAN_ID = 9; // TODO: change this
    private static final int ROD_CAN_ID = 8; // TODO: change this
    private static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;

    public static final Angle HOME_ANGLE = Degrees.of(345); // TODO: change this
    public static final Angle COLLECTION_POINT = Degrees.of(270); // TODO: change this

    private static final double aP = 0; // TODO: change this
    private static final double aI = 0; // TODO: change this
    private static final double aD = 0; // TODO: change this

    private static final double rP = 0; // TODO: change this
    private static final double rI = 0; // TODO: change this
    private static final double rD = 0; // TODO: change this

    private static final double CW_SPEED = 0; // TODO: change this
    private static final double CCW_SPEED = 0; // TODO: change this

    private static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(0); // TODO: change this
    private static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(0); // TODO: change this

    private static final Angle POSITION_TOLERANCE = Degrees.of(0); // TODO: change this
    private static final AngularVelocity VELOCITY_TOLERANCE = DegreesPerSecond.of(0); // TODO: change this

    public IntakeSubsystem() {

        CANcoderConfiguration armCanCoderConfig = new CANcoderConfiguration();
        armCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0; //TODO: Change
        armCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        armCanCoderConfig.MagnetSensor.MagnetOffset = 0.0;
        armCanMotor.getConfigurator().apply(armCanCoderConfig);

        TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
        armMotorConfig.Feedback.FeedbackRemoteSensorID = armCanMotor.getDeviceID();
        armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        armMotorConfig.Feedback.SensorToMechanismRatio = 0.0; //TODO: Change
        armMotorConfig.Feedback.RotorToSensorRatio = 0.0; //TODO: Change

        armMotor.getConfigurator().apply(armMotorConfig);

        armCanMotor= new CANcoder(ARM_CAN_ID);

        armMotor = new TalonFX(ARM_CAN_ID, "Placeholder");

        rodMotor = new TalonFX(ROD_CAN_ID, "Placeholder");

        armMotorConfig = new TalonFXConfiguration();

        rodMotorConfig = new TalonFXConfiguration();

        feedForward = new ArmFeedforward(
            0, // TODO: change this
            0, // TODO: change this
            0, // TODO: change this
            0 // TODO: change this
        );

        rodMotorConfig.Slot0.kP = (rP);
        rodMotorConfig.Slot0.kI = (rI);
        rodMotorConfig.Slot0.kD = (rD);
        
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
            VELOCITY_TOLERANCE.in(DegreesPerSecond)
        );

        setSetpoint(HOME_ANGLE);
            armPIDController.reset(getPosition().in(Degrees)
        );
    }

    public static Angle getPosition() {
        return Degrees.of(armCanMotor.getAbsolutePosition().getValueAsDouble());
    }

    public static AngularVelocity getVelocity() {
        return DegreesPerSecond.of(armCanMotor.getAbsolutePosition().getValueAsDouble());
    }

    public Command rotateTo(Angle setpoint) {
        return runOnce(
            () -> setSetpoint(setpoint)).until(
                () -> atGoal());
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
    
    public StatusSignal<Current> getDraw() {
        return rodMotor.getMotorStallCurrent();
    }
    
    private static void setSetpoint(Angle setpoint) {
        armPIDController.setGoal(setpoint.in(Degrees));
    }

    private boolean atGoal() {
        return armPIDController.atGoal();
    }

    @Override
    public void periodic() {
        armMotor.setVoltage(
            armPIDController.calculate(getPosition().in(Degrees)) +
                feedForward.calculate(
                    armPIDController.getSetpoint().position,
                    armPIDController.getSetpoint().velocity
                )
            );
    }

}
