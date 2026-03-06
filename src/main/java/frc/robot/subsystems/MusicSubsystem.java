package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.manipSubsystems.SpitterSubsystem;

public class MusicSubsystem extends SubsystemBase {
    SpitterSubsystem spitterSubsystem;

    public MusicSubsystem() {
        spitterSubsystem = new SpitterSubsystem();
    }
    Orchestra m_orchestra = new Orchestra();

    public static void musicSubsystem() {

        m_orchestra.addInstrument(leftSpitter);

    }
}
