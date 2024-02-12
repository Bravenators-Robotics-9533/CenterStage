package com.bravenatorsrobotics.utils;

import com.qualcomm.robotcore.util.Range;

public class ServoUtils {

    public static double convertStandardToContinualRange(double value) {

        value = Range.clip(value, -1.0, 1.0);
        return (value + 1.0) / 2.0;

    }

}
