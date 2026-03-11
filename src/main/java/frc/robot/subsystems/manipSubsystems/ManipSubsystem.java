package frc.robot.subsystems.manipSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ManipSubsystem extends SubsystemBase {
    IntakeSubsystem intakeSubsystem;
    IndexSubsystem indexSubsystem;
    SpitterSubsystem spitterSubsystem;
    
    public ManipSubsystem(DriveSubsystem drive) {
        intakeSubsystem = new IntakeSubsystem();
        spitterSubsystem = new SpitterSubsystem(drive);
        indexSubsystem = new IndexSubsystem(spitterSubsystem);
    }

    /**
     * runs intake roller
     */
    public Command intake() {
        return runOnce(() -> System.out.println("Intaking..."))
        .andThen(intakeSubsystem.intake());
    }

    /**
     * moves intake arm to be vertical
     */
    public Command stow() {
        return runOnce(() -> System.out.println("SSTOOOOOOOOWWWWWWWWWWWWWW"))
        .andThen(intakeSubsystem.stow())
        .andThen(runOnce(() -> System.out.println("WE ARE DONE WITH STOWING")));
    }

    /**
     * moves intake arm to be at the home limit
     */
    public Command home() {
        return runOnce(() -> System.out.println("Returning to home"))
        .andThen(intakeSubsystem.goHome())
        .andThen(runOnce(() -> System.out.println("At home!")));
    }

    /**
     * moves the intake arm out
     */
    public Command intakeArmOut() {
        return runOnce(() -> System.out.println("WE HAVE BEGUN THE PROCESS OF EXTENDING"))
        .andThen(intakeSubsystem.extendIntake())
        .andThen(runOnce(() -> System.out.println("WE ARE SO DONE WITH EXTENDING")));
    }

    /**
     * Spins the shooter wheel
     */
    public Command spinUp() {
        // return spitterSubsystem.setSpitterSpeed()
        // .andThen(spitterSubsystem.shoot());

        return spitterSubsystem.shoot();

        // return spitterSubsystem.manualShoot();
    }

    /**
     * 
     * runs intake and fires the balls
     */
    public Command shoot() {
        //AUTOSHOOT
        // return spitterSubsystem.setSpitterSpeed()
        // .andThen(indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1))
        // .andThen(new ParallelCommandGroup(
        //     intakeSubsystem.agitateFuel(),
        //     spitterSubsystem.shoot(),
        //     indexSubsystem.index(indexSubsystem.forwards)
        // ));

        //MANUAL ADJUSTABLE SHOOT
        return indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1)
        .andThen(new ParallelCommandGroup(
            intakeSubsystem.agitateFuel(),
            spitterSubsystem.shoot(),
            indexSubsystem.index(indexSubsystem.forwards)
        ));

        //MANUAL SHOOT
        // return indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1)
        // .andThen(new ParallelCommandGroup(
        //     intakeSubsystem.agitateFuel(),
        //     spitterSubsystem.manualShoot(),
        //     indexSubsystem.index(indexSubsystem.forwards)
        // ));
    }
    
    /** 
     * reverses the intake to blast out balls from intake
     */
    public Command extake() {
        return new ParallelCommandGroup(
            indexSubsystem.index(indexSubsystem.reverse),
            intakeSubsystem.extake()
        );
    }

    /**
     * pass within a close range
     */
    public Command lowPass() {
        return indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1)
        .andThen(new ParallelCommandGroup(
            intakeSubsystem.agitateFuel(),
            spitterSubsystem.lowPass(),
            indexSubsystem.index(indexSubsystem.forwards)
        ));
    }

    /**
     * pass from a decent distance
     */
    public Command midPass() {
        return indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1)
        .andThen(new ParallelCommandGroup(
            intakeSubsystem.agitateFuel(),
            spitterSubsystem.midPass(),
            indexSubsystem.index(indexSubsystem.forwards)
        ));
    }

    /**
     * pass from far away
     */
    public Command highPass() {
        return indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1)
        .andThen(new ParallelCommandGroup(
            intakeSubsystem.agitateFuel(),
            spitterSubsystem.highPass(),
            indexSubsystem.index(indexSubsystem.forwards)
        ));
    }

    public boolean shooterIsReady() {
        return spitterSubsystem.shooterIsReady();
    }

    public Command upSRPSCommand() {
        return spitterSubsystem.upSRPSCommand();
    }

    public Command downSRPSCommand() {
        return spitterSubsystem.downSRPSCommand();
    }

    public Command upFRPSCommand() {
        return spitterSubsystem.upFRPSCommand();
    }

    public Command downFRPSCommand() {
        return spitterSubsystem.downFRPSCommand();
    }

    /**
     * sets the spitter speed to spin up to based on distance from hub
     */
    public Command setSpitterSpeed() {
        return spitterSubsystem.setSpitterSpeed();
    }

}