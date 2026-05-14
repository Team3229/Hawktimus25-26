package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;


public class MusicSubsystem {

    Orchestra m_orchestra;
    StatusCode music;
    
    public MusicSubsystem(TalonFX talon) {

        m_orchestra = new Orchestra();

        m_orchestra.addInstrument(talon);

        music = m_orchestra.loadMusic("src/main/deploy/ImperialMarch.chrp");

        if(!music.isOK()) {
            System.out.println("Failed to load music: " + music.toString());
        } else {
            m_orchestra.play();
        }

    }

    // make this able to apply to all talons in a subsystem (not drive tho)
    
}
