package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
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

    private static TalonFXConfiguration rodMotorConfig;
    private static DigitalInput restLimitSwitch;
    private static DigitalInput outLimitSwitch;
    
    private static final int ARM_R_CAN_ID = 9; 
    private static final int ARM_L_CAN_ID = 1; 
    
    private static final int REST_LIMIT_PORT = 1; //TODO: update upon implementation
    private static final int OUT_LIMIT_PORT = 0; //TODO: update upon implementation
    
    private static final int ROD_CAN_ID = 2; 
    
    private static final Current CURRENT_LIMIT = Amps.of(40);
    
    public static final Angle HOME_ANGLE = Rotations.of(0);
    public static final Angle STOW_ANGLE = Rotations.of(0.097);
    public static final Angle COLLECTION_POINT = Rotations.of(0.347);
    
    public static final boolean inversion = false;

    private static double sensorToMechanismRatio = 0.04;
    
    private Angle requestedAngle;
    private double requestedVelocity;
    
    private static final double aP = 0.1;
    private static final double aI = 0; 
    private static final double aD = 0; 

    private static final double rP = 0.3; 
    private static final double rV = 0.13; 

    private static final double ROD_CW_SPEED = 50; 
    private static final double ROD_CCW_SPEED = -50;

    public IntakeSubsystem() {
        restLimitSwitch = new DigitalInput(REST_LIMIT_PORT);
       
        outLimitSwitch = new DigitalInput(OUT_LIMIT_PORT);

        armMotorRight = new TalonFX(ARM_R_CAN_ID, CANBus.roboRIO());

        armMotorLeft = new TalonFX(ARM_L_CAN_ID, CANBus.roboRIO());

        rodMotor = new TalonFX(ROD_CAN_ID, CANBus.roboRIO());
                      
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

        armMotorRight.getConfigurator().apply(armMotorConfig);

        // TEST 2 IS WITH SEPERATE CONFIG
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
        armMotorConfig2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        armMotorLeft.getConfigurator().apply(armMotorConfig2);


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

        outLimit().onTrue(
            Commands.runOnce(this::emergencyStow).ignoringDisable(true)
        );

        SmartDashboard.putData("Intake", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("PositionR", () -> armMotorRight.getPosition().getValueAsDouble(), null);
                builder.addDoubleProperty("PositionL", () -> armMotorLeft.getPosition().getValueAsDouble(), null);
                builder.addDoubleProperty("VelocityR", () -> armMotorRight.getVelocity().getValueAsDouble(), null);
                builder.addDoubleProperty("VelocityL", () -> armMotorLeft.getVelocity().getValueAsDouble(), null);
                builder.addBooleanProperty("ReadyToIntake", () -> armIsReady(), null);
            }
        });

        SmartDashboard.putData("IntakeRod", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Velocity", () -> rodMotor.getVelocity().getValueAsDouble(), null);
                builder.addBooleanProperty("Intaking", () -> rodIsReady(), null);
            }
        });
    }

    /**
     * Gets the angle in degrees of the arm from the CAN
     * 
     * @return The degree of the arm
     */
    public static Angle getPosition() {
        return Degrees.of(armMotorRight.getPosition().getValueAsDouble());
    }

    /**
     * Gets the angular velocity of the arm from the CAN
     * 
     * @return The velocity of the arm
     */
    public static AngularVelocity getVelocity() {
        return DegreesPerSecond.of(armMotorRight.getVelocity().getValueAsDouble());
    }

    /**
     * Gets the angular acceleration of the arm from the CAN
     * 
     * @return The acceleration of the arm
     */
    public static AngularAcceleration getAcceleration() {
        return DegreesPerSecondPerSecond.of(armMotorRight.getAcceleration().getValueAsDouble());
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
                System.out.println("Rotate initialize print");
            }

            @Override
            public void execute() {
                System.out.println("Rotating to " + setpoint.toShortString());
                armMotorRight.setControl(new MotionMagicDutyCycle(setpoint)
                .withSlot(0)
                .withFeedForward(0));
                armMotorLeft.setControl(new MotionMagicDutyCycle(setpoint)
                .withSlot(0)
                .withFeedForward(0));
            }

            @Override
            public boolean isFinished() {
                System.out.println("Checking done for rotate!!!!!!!!!!!!!!!!!!!!!!!!!");
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

            // @Override
            // public boolean isFinished() {
            //     if(outLimitSwitch()) {
            //         Commands.runOnce(() -> {
            //             CommandScheduler.getInstance().cancelAll();
            //         });
            //         return true;
            //     } else {
            //         return false;
            //     }
            // }
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

    /** gets your currrent current */
    public StatusSignal<Current> getDraw() {
        return rodMotor.getMotorStallCurrent();
    }

    private boolean armIsReady() {
        Angle deadBand = Rotations.of(0.01);
        Angle rightArmAngle = armMotorRight.getPosition().getValue();
        Angle leftArmAngle = armMotorLeft.getPosition().getValue();
        System.out.println("req: " + requestedAngle.toShortString() + " Rcur: " + rightArmAngle.toShortString() + " Lcur: " + leftArmAngle.toShortString());
        if (rightArmAngle.isNear(requestedAngle, deadBand) && leftArmAngle.isNear(requestedAngle, deadBand)) {
            return true;
        } else {
            return false;
        }
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

      public Trigger outLimit() {
        return new Trigger(this::outLimitSwitch);
    }

    private boolean outLimitSwitch() {
        return outLimitSwitch.get();
    }

      public Trigger restLimit() {
        return new Trigger(this::restLimitSwitch);
    }

    private boolean restLimitSwitch() {
        return restLimitSwitch.get();
    }
}