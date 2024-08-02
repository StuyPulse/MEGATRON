package com.stuypulse.robot.subsystems.leds.instructions;

import java.util.ArrayList;

import com.stuypulse.robot.util.SLColor;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED694 implements LEDInstruction {

    private final double pulsingTime;
    private final SLColor[] colorMap;
    private final ArrayList<Integer> colorArray;
    private final StopWatch stopWatch;

    public LED694(double pulsingTime, SLColor secondaryColor) {
        this.pulsingTime = pulsingTime;
        colorMap = new SLColor[] {SLColor.RED, secondaryColor};
        colorArray = initColors();
        stopWatch = new StopWatch();
    }

    public LED694() {
        this(0.03, SLColor.WHITE);
    }


    @Override
    public void setLED(AddressableLEDBuffer ledBuffer) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, colorMap[colorArray.get(i)]);
        }

        if (pulsingTime < stopWatch.getTime()){
            int firstColor = colorArray.get(0);
            for (int i = 1; i < colorArray.size(); i++){
                colorArray.set(i - 1, colorArray.get(i));
            }
            colorArray.set(colorArray.size() - 1, firstColor);

            stopWatch.reset();
        }
    }


    private static ArrayList<Integer> initColors() {
        ArrayList<Integer> colors = new ArrayList<>();
        for (int i = 0; i < 6; i++) colors.add(0);
        for (int i = 0; i < 6; i++) colors.add(1);
        for (int i = 0; i < 9; i++) colors.add(0);
        for (int i = 0; i < 9; i++) colors.add(1);
        for (int i = 0; i < 4; i++) colors.add(0);
        for (int i = 0; i < 4; i++) colors.add(1);
        return colors;
    }
    
}
