package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.drive.GameState;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    
    // THE LEDS ARE IN BGR NOT RGB
    private final Color purple = new Color(255, 0, 255);
    private final Color yellow = new Color(0, 100, 255);

    private final int LEDPortNumber = 1;
    private final int LEDBuffer = 60;

    public LEDSubsystem() {
        led = new AddressableLED(LEDPortNumber);

        ledBuffer = new AddressableLEDBuffer(LEDBuffer);    

        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
       
        led.start();
       
        this.setDefaultCommand(hubActive());
       
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }

    public static Color BGRPacker(int red, int green, int blue) {
        return new Color(blue, green, red);
    }

    public Command hubActive() {
        if (GameState.isMyHubActive() == true) {
            return runPattern(setHawkBlink());
        } else {
            return runPattern(setHawkOffset());
        }
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(ledBuffer));
    }

    public LEDPattern setColor(Color color) {
        return LEDPattern.solid(color);
    }

    public LEDPattern setPurple() {
        return setColor(purple);
    }

    public LEDPattern setColorsContinuous(Color c1, Color c2) {
        return LEDPattern.gradient(GradientType.kContinuous, c1, c2);
    }

    public LEDPattern setHawkContinuous() {
        return setColorsContinuous(purple, yellow);
    }

    public LEDPattern setColorsSteps(Color c1, Color c2) {
       return LEDPattern.steps(Map.of(0, c1, 0.5, c2));
    }

    public LEDPattern setHawkSteps() {
        return setColorsSteps(purple, yellow);
    }
   
    public LEDPattern setColorProgressMask(Color forProgressMask, Color secondForProgressMask, double currentHeight, double maxHeight) {
        return LEDPattern.progressMaskLayer(() -> {return (currentHeight / maxHeight);});
    }

    public LEDPattern setHawkMask(double currentHeight, double maxHeight) {
        return setColorProgressMask(purple, yellow, currentHeight, maxHeight);
    }

    public LEDPattern setColorOffset(Color colorForOffset, Color secondColorForOffset) {
        return setDiscontGradient(colorForOffset, secondColorForOffset).offsetBy(LEDBuffer/10);
    }

    public LEDPattern setHawkOffset() {
        return setColorOffset(purple, yellow);
    }

    public LEDPattern setColorReverse(Color colorForReverse, Color secondColorForReverse) {
        return setDiscontGradient(colorForReverse, secondColorForReverse).reversed();
    }

    public LEDPattern setHawkReverse() {
        return setColorReverse(purple, yellow);
    }

    // public LEDPattern setColorScroll(Color colorForScroll, Color secondColorForScroll) {
    //     return setDiscontGradient(colorForScroll, secondColorForScroll).scrollAtRelativeSpeed(new Frequency.of(1));
    // }

    // public LEDPattern setHawkScroll() {
    //     return setColorScroll(purple, yellow);
    // }

    public LEDPattern setColorBreathe(Color colorForBreathe, Color secondColorForBreathe) {
        return setDiscontGradientHawk().breathe(Seconds.of(3));
    }

    public LEDPattern setHawkBreathe() {
        return setColorBreathe(yellow, purple);
    }

    public LEDPattern setColorBlink(Color colorForBlink, Color secondColorForBlink) {
        return setDiscontGradient(colorForBlink, secondColorForBlink).blink(Seconds.of(0.4));
    }

    public LEDPattern setHawkBlink() {
        return setColorBlink(purple, yellow);
    }

    public LEDPattern setColorBrightness(Color colorForBrightness, Color secondColorForBrightness, double constantBrightnessLevelIsCool) {
        return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,colorForBrightness, secondColorForBrightness);
    }

    public LEDPattern setHawkBrightness(double brightnessLevel) {
        return setColorBrightness(purple, yellow, brightnessLevel);
    }

    public LEDPattern setColorMask(Color colorForMask, Color secondColorForMask) {
        return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colorForMask, secondColorForMask);
    }

    public LEDPattern setHawkMask() {
        return setColorMask(purple, yellow);
    }

    public LEDPattern setDiscontGradient(Color c1, Color c2) {
        return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, c1, c2);
    }

    public LEDPattern setDiscontGradientHawk() {
        return setDiscontGradient(purple, yellow);
    }
}