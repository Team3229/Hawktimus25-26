package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
   
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private LEDPattern pattern;

    private final Color purple = Color.fromHSV(300, 100, 50);
    private final Color yellow = Color.fromHSV(60,100,100);

    private final int LEDPortNumber = 3;
    private final int LEDBuffer = 180;

    public LEDSubsystem() {
        led = new AddressableLED(LEDPortNumber);
        ledBuffer = new AddressableLEDBuffer(LEDBuffer);

        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();

        this.setDefaultCommand(runPattern(setColor(Color.kBlue)));

    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(ledBuffer));
    }

    public LEDPattern setColor(Color color) {
        return LEDPattern.solid(color);
    }

    public void setColorsContinuous(Color c1, Color c2) {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, c1, c2);
        pattern.applyTo(ledBuffer);
    }

    public Command setPurpleAndYellowContinuous() {
        return Commands.run(() -> setColorsContinuous(purple, yellow));
    }

    public Command setColorsDiscontinuous(Color c1, Color c2) {
        return Commands.runOnce(() -> pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, c1, c2))
        .andThen(() -> pattern.applyTo(ledBuffer));
    }

    public Command setPurpleandYellowDiscontinuous() {
        return setColorsDiscontinuous(purple, yellow);
    }

    public void setColorsSteps(Color c1, Color c2) {
        pattern = LEDPattern.steps(Map.of(0, c1, 0.5, c2));
        pattern.applyTo(ledBuffer);
    }

    public void setPurpleandYellowSteps() {
        setColorsSteps(purple, yellow);
    }
   
    public void setColorProgressMask(Color forProgressMask, Color secondForProgressMask) {
        pattern = LEDPattern.progressMaskLayer(null);
        pattern.applyTo(ledBuffer);
    }

    public void setPurpleandYellowProgressMask() {
        setColorProgressMask(purple, yellow);
    }

    public void setColorOffset(Color colorForOffset, Color secondColorForOffset) {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colorForOffset, secondColorForOffset);
        pattern = pattern.offsetBy(40);
        pattern = pattern.offsetBy(-20);
        pattern.applyTo(ledBuffer);
    }

    public void setPurpleandYellowOffset() {
        setColorOffset(purple, yellow);
    }

    public void setColorReverse(Color colorForReverse, Color secondColorForReverse) {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colorForReverse, secondColorForReverse);
        pattern = pattern.reversed();
        pattern.applyTo(ledBuffer);
    }

    public void setPurpleandYellowReverse() {
        setColorReverse(purple, yellow);
    }

    public void setColorScroll(Color colorForScroll, Color secondColorForScroll) {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colorForScroll, secondColorForScroll);
        pattern = pattern.scrollAtRelativeSpeed(Percent.per(Second).of(25));
        pattern = pattern.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), Meters.of(1.0/84));
        pattern.applyTo(ledBuffer);
    }

    public void setPurpleandYellowScroll() {
        setColorScroll(purple, yellow);
    }

    public void setColorBreathe(Color colorForBreathe, Color secondColorForBreathe) {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colorForBreathe, secondColorForBreathe);
        pattern = pattern.breathe(Seconds.of(2));
        pattern.applyTo(ledBuffer);
    }

    public void setPurpleandYellowBreathe() {
        setColorBreathe(purple, yellow);
    }

    public void setColorBlink(Color colorForBlink, Color secondColorForBlink) {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colorForBlink, secondColorForBlink);
        pattern = pattern.blink(Seconds.of(1.5));
        pattern = pattern.blink(Seconds.of(2), Seconds.of(1));
        pattern = pattern.synchronizedBlink(RobotController::getRSLState);
        pattern.applyTo(ledBuffer);
    }

    public void setPurpleandYellowBlink() {
        setColorBlink(purple, yellow);
    }

    public void setColorBrightness(Color colorForBrightness, Color secondColorForBrightness, double constantBrightnessLevelIsCool) {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,colorForBrightness, secondColorForBrightness);
        pattern = pattern.atBrightness(Percent.of(constantBrightnessLevelIsCool));
        pattern.applyTo(ledBuffer);

    }

    public void setPurpleandYellowBrightness(double brightnessLevel) {
        setColorBrightness(purple, yellow, brightnessLevel);
    }

    public void setColorMask(Color colorForMask, Color secondColorForMask) {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,colorForMask, secondColorForMask);
        pattern = LEDPattern.progressMaskLayer(null);
        pattern = pattern.mask(pattern);
        pattern.applyTo(ledBuffer);
    }

    public void setPurpleandYellowColorMask() {
        setColorMask(purple, yellow);
    }

    public void setColorAnimatedMask(Color colorForAnimatedMask, Color secondColorForAnimatedMask) {
        Map<? extends Number, Color> maskSteps = Map.of(0, colorForAnimatedMask,0.5, secondColorForAnimatedMask);
        pattern = LEDPattern.rainbow(255,255);
        pattern = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(0.25));
        pattern = pattern.mask(pattern);
        pattern.applyTo(ledBuffer);
    }

    public void setPurpleandYellowAnimatedMask() {
        setColorAnimatedMask(purple, yellow);
    }







    // public Command hasFuel() {
    //      Command out = new Command() {
    //         @Override public void execute() {
    //             setAllLEDs(new Color(255, 0, 0));
    //         }

    //         @Override public String getName() {
    //             return "ledNote";
    //         }
    //     };

    //     out.addRequirements(this);

    //     return out;
    // }
}


