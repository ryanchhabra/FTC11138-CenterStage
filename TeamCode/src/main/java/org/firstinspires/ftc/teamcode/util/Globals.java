package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {

    public static boolean LIMITS = true;

    public static boolean IS_AUTO = false;
    public static PropLocation PROP_LOCATION = PropLocation.NONE;
    public static ParkSide PARK_SIDE = ParkSide.MIDDLE;

    public static Alliance ALLIANCE;
    public static Side SIDE;


    public static boolean IS_SCORING = false;
    public static boolean IS_INTAKING = false;

    public static void startScoring() {
        IS_SCORING = true;
        IS_INTAKING = false;
    }

    public static void stopScoring() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }

    public static void startIntaking() {
        IS_SCORING = false;
        IS_INTAKING = true;
    }

    public static void stopIntaking() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }


    public enum PropLocation {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    public enum Alliance {
        RED,
        BLUE
    }

    public enum Side {
        BACKDROP,
        AUDIENCE
    }

    public enum ParkSide {
        MIDDLE,
        CORNER
    }

}
