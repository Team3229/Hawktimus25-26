package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeSubsystem extends SubsystemBase {

    private static TalonFX armMotorRight;
    private static TalonFX armMotorLeft;
    private static TalonFX rodMotor;
    private static TalonFXConfiguration armMotorConfig;
    private static TalonFXConfiguration armMotorConfig2;

    private static MotionMagicVoltage rotateRequest;

    private static double motionMagicCruiseVelocity = 0.5;
    private static double motionMagicAcceleration = 1;
    private static double motionMagicJerk = 10;

    private static TalonFXConfiguration rodMotorConfig;

    private static DigitalInput homeLimitSwitch;
    private static DigitalInput extendLimitSwitch;

    // private static Angle softLimitUp = Rotations.of(-0.07);
    // private static Angle softLimitDown = Rotations.of(0.37);
    
    private static final int ARM_R_CAN_ID = 9; 
    private static final int ARM_L_CAN_ID = 1; 
    
    private static final int HOME_LIMIT_PORT = 1; 
    private static final int EXTEND_LIMIT_PORT = 0;
    
    private static final int ROD_CAN_ID = 2; 
    
    private static final Current CURRENT_LIMIT = Amps.of(40);
    
    public static final Angle HOME_ANGLE = Rotations.of(0);
    public static final Angle STOW_ANGLE = Rotations.of(0.097);
    public static final Angle COLLECTION_POINT = Rotations.of(0.347);
    
    private static double sensorToMechanismRatio = 25;
    
    private Angle requestedAngle;
    private double requestedVelocity;
    
    private static final double aP = 4.8;
    private static final double aI = 0; 
    private static final double aD = 0.1; 
    private static final double aV = 0.12;
    private static final double aA = 0.01;
    private static final double aS = 0.25;


    private static final double rP = 0.3; 
    private static final double rV = 0.13; 
    
    private static final double ROD_CW_SPEED = 50; 
    private static final double ROD_CCW_SPEED = -50;

    private static final Angle angleDeadBand = Rotations.of(0.01);
    
    public IntakeSubsystem() {
        super();

        homeLimitSwitch = new DigitalInput(HOME_LIMIT_PORT);
        
        extendLimitSwitch = new DigitalInput(EXTEND_LIMIT_PORT);
        
        armMotorRight = new TalonFX(ARM_R_CAN_ID, CANBus.roboRIO());
        
        armMotorLeft = new TalonFX(ARM_L_CAN_ID, CANBus.roboRIO());
        
        rodMotor = new TalonFX(ROD_CAN_ID, CANBus.roboRIO());

        rotateRequest = new MotionMagicVoltage(0).withSlot(0);

        armMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(
        new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true)
        )
        .withFeedback(
            new FeedbackConfigs()
            .withSensorToMechanismRatio(sensorToMechanismRatio)
        );
                        
        armMotorConfig.Slot0.kP = aP;
        armMotorConfig.Slot0.kI = aI;
        armMotorConfig.Slot0.kD = aD;
        armMotorConfig.Slot0.kV = aV;
        armMotorConfig.Slot0.kA = aA;
        armMotorConfig.Slot0.kS = aS;
        armMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        armMotorConfig.MotionMagic
        .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
        .withMotionMagicAcceleration(motionMagicAcceleration)
        .withMotionMagicJerk(motionMagicJerk);

        armMotorRight.getConfigurator().apply(armMotorConfig);
        
        armMotorConfig2 = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
        )
        .withFeedback(
            new FeedbackConfigs()
            .withSensorToMechanismRatio(sensorToMechanismRatio)
        );

        armMotorConfig2.Slot0.kP = aP;
        armMotorConfig2.Slot0.kI = aI;
        armMotorConfig2.Slot0.kD = aD;
        armMotorConfig2.Slot0.kV = aV;
        armMotorConfig2.Slot0.kA = aA;
        armMotorConfig2.Slot0.kS = aS;
        armMotorConfig2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        armMotorConfig2.MotionMagic
        .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
        .withMotionMagicAcceleration(motionMagicAcceleration)
        .withMotionMagicJerk(motionMagicJerk);

        armMotorLeft.getConfigurator().apply(armMotorConfig2);

        armMotorRight.setControl(new Follower(ARM_L_CAN_ID, MotorAlignmentValue.Opposed));

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

        extendLimit().onTrue(
            Commands.runOnce(() -> {
                setExtended();
            })
        );

        homeLimit().onTrue(
            Commands.runOnce(() -> {
                System.out.println("Home limit hit, stopping motors and reseting position");
                setHome();
            })
        );

        setHome();

        this.setDefaultCommand(Commands.runOnce(() -> armMotorLeft.set(0)));

    }
    
    /** rotates the arm to the angle, finishes when armIsReady returns true */
    public Command rotateTo(Angle setpoint) {
        Command out = new Command() {
            @Override
            public void initialize() {
                requestedAngle = setpoint;
                System.out.println("Rotate initialize print");
            }

            @Override
            public void execute() {
                System.out.println("Rotating to " + setpoint.toShortString());
                
                armMotorLeft.setControl(rotateRequest.withPosition(setpoint));

            }

            @Override
            public boolean isFinished() {
                System.out.println("Checking done for rotate!!!!!!!!!!!!!!!!!!!!!!!!!");
                return armIsReady();
            }

            // @Override
            // public void end(boolean interrupted) {
            //     armMotorLeft.set(0);
            // }
        };

        initSendable();

        out.addRequirements(this);
        return out;
    
    }

    public Command intakeExtake(double speedSetpoint) {
        Command out = new Command() {
             @Override 
             public void initialize() {
                 requestedVelocity = speedSetpoint;
             }
             @Override
             public void execute() {
                rodMotor.setControl(new VelocityVoltage(speedSetpoint).withSlot(0));

             }
             @Override
             public void end(boolean interrupted) {
                rodMotor.setControl(new VelocityVoltage(0).withSlot(0));
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
       return intakeExtake(ROD_CCW_SPEED);
    }
      /**
     * creates a command to reverse the intake to put the fuel into the human player
     * station
     * 
     * @return Command to spin the rod in reverse
     */ 
    public Command extake() {
        return intakeExtake(ROD_CW_SPEED);
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
     * Command to return to a safe angle after hitting limit
     */
    public Command emergencyStow() {
        return rotateTo(STOW_ANGLE);
    }

    /**
     * creates a command that pulls the intake arm back to the
     * home point in order to move the fuel in storage.
     * 
     * @return Command to pull arm back
     */
    public Command agitateFuel() {
        return rotateTo(STOW_ANGLE)
            .andThen(new WaitCommand(0.5))
            .andThen(rotateTo(COLLECTION_POINT)
        );
    }

    /** returns true when arm is within deadband */
    private boolean armIsReady() {
        if (requestedAngle == null) {
            return false;
        }
        // Angle rightArmAngle = armMotorRight.getPosition().getValue();

        Angle leftArmAngle = armMotorLeft.getPosition().getValue();

        // System.out.println("req: " + requestedAngle.toShortString() + " Rcur: " + rightArmAngle.toShortString() + " Lcur: " + leftArmAngle.toShortString());

        return leftArmAngle.isNear(requestedAngle, angleDeadBand);
    }

    private boolean rodIsReady() {
        double deadBand = 1;
        double rodVelocity = rodMotor.getVelocity().getValueAsDouble();
        if(requestedVelocity == 0) {
            return false;
        } else {
          return Math.abs(requestedVelocity - rodVelocity) <= deadBand;
        }
    }

    public Trigger extendLimit() {
        return new Trigger(this::extendLimitSwitch);
    }

    private boolean extendLimitSwitch() {
        return extendLimitSwitch.get();
    }

      public Trigger homeLimit() {
        return new Trigger(this::homeLimitSwitch);
    }

    private boolean homeLimitSwitch() {
        return homeLimitSwitch.get();
    }

    public void initSendable() {
        SmartDashboard.putData("Intake", new Sendable() {
            @Override 
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("PositionL", () -> armMotorLeft.getPosition().getValueAsDouble(), null);
                builder.addDoubleProperty("PositionR", () -> armMotorRight.getPosition().getValueAsDouble(), null);
                builder.addDoubleProperty("VelocityL", () -> armMotorLeft.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("VelocityR", () -> armMotorRight.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("VoltageL", ()-> armMotorLeft.getMotorVoltage().getValueAsDouble(), null); 
                builder.addDoubleProperty("VoltageR", ()-> armMotorRight.getMotorVoltage().getValueAsDouble(), null); 
                builder.addBooleanProperty("Arm is ready", () -> armIsReady(), null);
                builder.addBooleanProperty("Intaking", () -> rodIsReady(), null);
                builder.addDoubleProperty("Velocity", () -> rodMotor.getVelocity().getValueAsDouble(), null);
                builder.addBooleanProperty("Intaking", ()-> rodIsReady(), null); 

                }
        });
    }

    public void setHome() {
        armMotorLeft.set(0);
        armMotorLeft.setPosition(HOME_ANGLE);
        armMotorRight.setPosition(HOME_ANGLE);
        System.out.println("Home has been set");
    }

    public void setExtended() {
        armMotorLeft.set(0);
        armMotorLeft.setPosition(COLLECTION_POINT);
        armMotorRight.setPosition(COLLECTION_POINT);
        System.out.println("Extend limit hit, stopping motors and reseting position");
    }

    public void periodic() {

    }
}