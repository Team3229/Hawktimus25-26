package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;


public class MusicSubsystem {

    Orchestra m_orchestra = new Orchestra();
    
    public MusicSubsystem(TalonFX talon) {
        
        m_orchestra.addInstrument(talon);

        var music = m_orchestra.loadMusic("src/main/deploy/ImperialMarch.chrp");

        if(!status.isOK()) {
            System.out.println("Failed to load music: " + status.toString());
        } else {
            m_orchestra.play();
        }

    }
    
}
