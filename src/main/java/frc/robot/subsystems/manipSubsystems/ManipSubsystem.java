package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class ManipSubsystem extends SubsystemBase {
    
    IntakeSubsystem intakeSubsystem;
    IndexSubsystem indexSubsystem;
    ShooterArmSubsystem shooterArmSubsystem;
    SpitterSubsystem spitterSubsystem;
    
        
        boolean intakeOut = false;
        boolean shooterSpinning = false;
        boolean shooterMaxSpeed = false; 

    public void manipSubsystems() {
        intakeSubsystem = new IntakeSubsystem();
        indexSubsystem = new IndexSubsystem();
        shooterArmSubsystem = new ShooterArmSubsystem();
        spitterSubsystem = new SpitterSubsystem();

        SmartDashboard.putBoolean("Intake Arm Down", indexing);
        
       
        SmartDashboard.putBoolean("Intakeing", indexing);
    }

    public Command intake() {
        return runOnce(() -> System.out.println("Intaking...")) 
        .andThen(intakeSubsystem.intake())
        .andThen(Commands.waitTime(Seconds.of(2)))
        .andThen(indexSubsystem.index(indexSubsystem.forwards)); // might not be needed
        
    }

    SmartDashboard.putBoolean("Intake Arm Down", false);

    public Command extendStorage() {
        return runOnce(() -> intakeSubsystem.extendIntake());
    }
    
    public Command spinUp() {
        return runOnce(() -> spitterSubsystem.spit(1 /*placeholder will be replaced with distance*/));
    }

    public Command shoot() {
        return runOnce(() -> indexSubsystem.index(indexSubsystem.forwards));
    }

    public Command extake() {
        return runOnce(() -> intakeSubsystem.extake())
        .andThen(indexSubsystem.index(indexSubsystem.reverse));
    }
}
