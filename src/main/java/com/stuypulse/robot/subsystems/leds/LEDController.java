package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private static LEDController instance;

    static {
        instance = new LEDController(Ports.LEDController.PORT, Settings.LED.LED_LENGTH);
    }

    public static LEDController getInstance() {
        return instance;
    }

    private AddressableLED leds;
    private AddressableLEDBuffer ledsBuffer;

    protected LEDController(int port, int length) {
        leds = new AddressableLED(port);
        ledsBuffer = new AddressableLEDBuffer(length);

        leds.setLength(ledsBuffer.getLength());
        leds.setData(ledsBuffer);
        leds.start();

        runLEDInstruction(LEDInstructions.DEFAULT);

        SmartDashboard.putData(instance);
    }

    public void runLEDInstruction(LEDInstruction instruction) {
        instruction.setLED(ledsBuffer);
        leds.setData(ledsBuffer);
    }
}
