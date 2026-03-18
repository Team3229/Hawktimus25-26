package  frc.robot.subsystems.drive;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.manipSubsystems.SpitterSubsystem;
import frc.robot.utilities.LimelightHelpers;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class DriveConstants {
   private static final Slot0Configs anglePID = new Slot0Configs()
      .withKP(0).withKI(0).withKD(0)
      .withKS(0).withKV(0).withKA(0);

   private static final Slot0Configs drivePID = new Slot0Configs()
      .withKP(0).withKI(0).withKD(0)
      .withKS(0).withKV(0).withKA(0);
   
   private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
   private static final ClosedLoopOutputType kAngleClosedLoopOutput = ClosedLoopOutputType.Voltage;

   private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
   private static final DriveMotorArrangement kAngleMotorType = DriveMotorArrangement.TalonFX_Integrated;
   
   public boolean canFly = true; //if you delete this you like eating puppies or if your name is logan - Owen


   private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;
   
   private static final Current kSlipCurrent = Amps.of(40); //must be tuned

   private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(
         new CurrentLimitsConfigs()
           .withSupplyCurrentLimit(Amps.of(39))
           .withSupplyCurrentLimitEnable(true)
      );

   private static final TalonFXConfiguration angleInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(
         new CurrentLimitsConfigs()
           .withSupplyCurrentLimit(Amps.of(20))
           .withSupplyCurrentLimitEnable(true)
      );
      
   private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

   private static final Pigeon2Configuration pigeonConfigs;

   public static final CANBus kCANBus = new CANBus().roboRIO();

   public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5);
   public static final AngularVelocity kSpeedat12VoltsA = RotationsPerSecond.of(433.795);

    private static final double kCoupleRatio = 0; //Change later

    private static final double kDriveGearRatio = 0; //Change later
    private static final double kSteerGearRatio = 0; //Change later
    private static final Distance kWheelRadius = Inches.of(0); //Change later

    private static final boolean kInvertLeftSide = false; //Change later
    private static final boolean kInvertRightSide = true; //Change later

    private static final int kPigeonId = 1; //Change later
   
}