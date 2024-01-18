package org.firstinspires.ftc.teamcode.TestingClasses.ConfigClasses;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OpenCvHSVTester {

    // Use https://hyperskill.org/learn/step/13179 to calculate HSV values.
    // Make sure to convert to RGB (yeah it's confusing, but I was to lazy to
    // change it to RGB after realizing the proper labeling.

    public static int upperHue;
    public static int upperSaturation;
    public static int upperValue;

    public static int lowerHue;
    public static int lowerSaturation;
    public static int lowerValue;

    public OpenCvHSVTester(){

    }

    public int getUpperHue(){
        return upperHue;
    }

    public int getUpperSaturation(){
        return upperSaturation;
    }

    public int getUpperValue(){
        return upperValue;
    }

    public int getLowerHue(){
        return lowerHue;
    }

    public int getLowerSaturation(){
        return lowerSaturation;
    }

    public int getLowerValue(){
        return lowerValue;
    }
}
