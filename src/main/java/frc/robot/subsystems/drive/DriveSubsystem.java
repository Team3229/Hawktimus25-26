package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.manipSubsystems.SpitterSubsystem;
import frc.robot.utilities.LimelightHelpers;
import  frc.robot.subsystems.drive.DriveConstants;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// TODO:
// Initilase talon motors for drive
// Create a field relative drive
// Take the commands from yagsl file and make ctre: update odometry, reset odometry, get and set IMU yaw, getPose, zeroGyro, zeroGyroWithLimelight
// then steal: getTargetTranslation, angle from hub, distance from hub, hubAlign if(hubAlign)
// Make drive controller run all of them

public class DriveSubsystem extends SubsystemBase {
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(DriveConstants.kMaxSpeed.times(0.1))
        .withRotationalDeadband(DriveConstants.kMaxAngularRate.times(0.1))
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position);

    private final FieldCentric fieldcentric = new FieldCentric();

    // public StatusCode apply(SwerveControlParameters parameters, SwerveModule swerveModule) {}

    public final DriveConstants drivetrain = DriveConstants.createDrivetrain();
    
    private static final Rotation2d bluePerspRot = Rotation2d.kZero;
    private static final Rotation2d redPerspRot = Rotation2d.k180deg;
    private boolean operatorPersp = false; // TODO: determine if we need this

    public DriveSubsystem() {
        drivetrain.configNeutralMode(NeutralModeValue.Coast);
        driveTrain.seedFieldCentric(/* find out how to get rotation */);
    }

    @Override
    public void periodic() {

        // is  there a reason we are not running the reset odomenty from YAGSL subsystem?
        if (!operatorPersp || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? redPerspRot : bluePerspRot
                );
                operatorPersp = true;
            });
        }
    }

    public void updateOdometry() {
    //   public final SwerveDriveState john;//getState()
      var state = SwerveDrivetrain.SwerveDriveState getState();
      final Pose2d pose = state.Pose();
    }

   //I apologize greyson but this command is teleporting me to it asking me to import stuff evry 3 seconds
   
   
   
   
   
   
   
   
    //all errors are becaue owen is gay and short
}
