package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ManipSubsystem extends SubsystemBase {
    IntakeSubsystem intakeSubsystem;
    IndexSubsystem indexSubsystem;
    ShooterArmSubsystem shooterArmSubsystem;
    SpitterSubsystem spitterSubsystem;

    public ManipSubsystem() {
        intakeSubsystem = new IntakeSubsystem();
        indexSubsystem = new IndexSubsystem();
        shooterArmSubsystem = new ShooterArmSubsystem();
        spitterSubsystem = new SpitterSubsystem();
    }

    public Command intake() {
        return runOnce(() -> System.out.println("Intaking...")) 
        .andThen(intakeSubsystem.intake())
        .andThen(Commands.waitTime(Seconds.of(2)))
        .andThen(indexSubsystem.index(indexSubsystem.forwards)); // might not be needed
    }

    public Command extendStorage() {
        return runOnce(() -> intakeSubsystem.extendIntake());
    }
    
    public Command spinUp() {
        return runOnce(() -> spitterSubsystem.spit(1 /*placeholder will be replaced with distance*/));
    }

    public Command shoot() {
        return runOnce(() -> indexSubsystem.index(indexSubsystem.forwards))
        .andThen(intakeSubsystem.agitateFuel());
    }

    public Command extake() {
        return runOnce(() -> intakeSubsystem.extake())
        .andThen(indexSubsystem.index(indexSubsystem.reverse));
    }
}