package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicDutyCycle_Position;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.ExternalEncoderConfig.Presets;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeSubsystem extends SubsystemBase {

    private static TalonFX armMotorRight;
    private static TalonFX armMotorLeft;
    private static TalonFX rodMotor;
    private static TalonFXConfiguration armMotorConfig;
    private static TalonFXConfiguration rodMotorConfig;

    private static Encoder armEncoder;
    private static ExternalEncoderConfig armEncoderConfig;

    private static final int ARM_R_CAN_ID = 9; 
    private static final int ARM_L_CAN_ID = 1; 

    private static final int ROD_CAN_ID = 2; 

    private static final Current CURRENT_LIMIT = Amps.of(40);

    public static final Angle HOME_ANGLE = Rotations.of(0);
    public static final Angle COLLECTION_POINT = Rotations.of(0.25);

    public static final boolean inversion = false;

    private Angle requestedAngle;
    private double requestedVelocity;

    private static final double aP = 0.5; 
    private static final double aI = 0; 
    private static final double aD = 0; 

    private static final double rP = 0.3; 
    private static final double rV = 0.13; 

    private static final double ROD_CW_SPEED = 50; 
    private static final double ROD_CCW_SPEED = -50;

    public IntakeSubsystem() {

        armMotorRight = new TalonFX(ARM_R_CAN_ID, CANBus.roboRIO());

        armMotorLeft = new TalonFX(ARM_L_CAN_ID, CANBus.roboRIO());

        rodMotor = new TalonFX(ROD_CAN_ID, CANBus.roboRIO());

        armEncoderConfig = new ExternalEncoderConfig.Presets().REV_ThroughBoreEncoder;

        armEncoder = new Encoder(null /*need to learn what this is asking*/, /*need to learn*/, inversion);
                      
        armMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
        );

        armMotorConfig.Slot0.kP = aP;
        armMotorConfig.Slot0.kI = aI;
        armMotorConfig.Slot0.kD = aD;

        armMotorConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
        armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        armMotorConfig.Feedback.SensorToMechanismRatio = 0.04;
        armMotorConfig.Feedback.RotorToSensorRatio = 25.0;

        armMotorRight.getConfigurator().apply(armMotorConfig);
        armMotorLeft.getConfigurator().apply(armMotorConfig);
        armMotorLeft.setControl(new Follower(ARM_R_CAN_ID, MotorAlignmentValue.Opposed));

        rodMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(
                new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
            );

        rodMotorConfig.Slot0.kP = (rP);
        rodMotorConfig.Slot0.kV = (rV);

    }

    /**
     * Gets the angle in degrees of the arm from the CAN
     * 
     * @return The degree of the arm
     */
    public static Angle getPosition() {
        return Degrees.of(armEncoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Gets the angular velocity of the arm from the CAN
     * 
     * @return The velocity of the arm
     */
    public static AngularVelocity getVelocity() {
        return DegreesPerSecond.of(armEncoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Gets the angular acceleration of the arm from the CAN
     * 
     * @return The acceleration of the arm
     */
    public static AngularAcceleration getAcceleration() {
        return DegreesPerSecondPerSecond.of(armEncoder.getAbsolutePosition().getValueAsDouble());
    }

    // /**
    //  * Command that rotates the arm to a setpoint
    //  * 
    //  * @return Command to rotate arm
    //  */
    // public Command rotateTo(Angle setpoint) {
    //     requestedAngle = setpoint;
    //     return runOnce(
    //         () -> setSetpoint(setpoint)
    //     ).until(
    //         () -> armIsReady()
    //     );
    // }

    public Command rotateTo(Angle setpoint) {
        Command out = new Command() {
            @Override
            public void initialize() {
                requestedAngle = setpoint;
            }

            @Override
            public void execute() {
                armMotorRight.setControl(new Diff_MotionMagicDutyCycle_Position(
                    new MotionMagicDutyCycle(setpoint).withSlot(0),
                    new PositionDutyCycle(setpoint).withSlot(0)
                ));
            }

            @Override
            public boolean isFinished() {
                return armIsReady();
            }
        };

        out.addRequirements(this);
        return out;
    
    }

    /**
    * creates a command to collect the fuel by spinning the rod
    * 
    * @return Command to spin rod
    */
    public Command intake() {
        Command out = new Command() {
            @Override
            public void initialize() {
                requestedVelocity = ROD_CW_SPEED;
            }

            @Override
            public void execute() {
                rodMotor.setControl(new VelocityVoltage(ROD_CW_SPEED).withSlot(0).withFeedForward(12)); 
            }

            @Override
            public void end(boolean interrupted) {
                rodMotor.setControl(new VelocityVoltage(0).withSlot(0)); 
            }
        };
        
        SmartDashboard.putData("Intake", new Sendable() {
            @Override 
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Indexing", ()-> rodIsReady(), null); 
            }
        });

        out.addRequirements(this);
        return out;
    }

    /**
     * creates a command to reverse the intake to put the fuel into the human player
     * station
     * 
     * @return Command to spin the rod in reverse
     */
    public Command extake() {
        Command out = new Command() {
            @Override
            public void execute() {
                rodMotor.setControl(new VelocityVoltage(ROD_CCW_SPEED).withSlot(0).withFeedForward(Volts.of(12)));
            }
            @Override
            public void end(boolean interrupted) {
                rodMotor.setControl(new VelocityVoltage(0).withSlot(0).withFeedForward(Volts.of(12)));
            }
        };
        
        out.addRequirements(this);
        return out;
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
            .andThen(rotateTo(COLLECTION_POINT)
        );
    }

    /** gets your currrent current */
    public StatusSignal<Current> getDraw() {
        return rodMotor.getMotorStallCurrent();
    }

    private boolean armIsReady() {
        double deadBand = 0.01;
        double armAngle = armMotorRight.getPosition().getValueAsDouble();
        return Math.abs(((BaseStatusSignal) requestedAngle).getValueAsDouble() - armAngle) <= deadBand;
     
    }
    private boolean rodIsReady() {
        double deadBand = 1;
        double rodVelocity = rodMotor.getVelocity().getValueAsDouble();
        if(deadBand == 0) {
            return false;
        } else {
          return Math.abs(requestedVelocity - rodVelocity) <= deadBand;
        }
    }
}