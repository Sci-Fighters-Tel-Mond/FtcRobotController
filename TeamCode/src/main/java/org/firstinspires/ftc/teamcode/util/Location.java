package org.firstinspires.ftc.teamcode.util;

public class Location {
    public enum LOCATION {
        BLUE_EXTERNAL_START_POSITION, //(-1.75, 0)
        BLUE_INTERNAL_START_POSITION, //(-0.75, 0)
        RED_EXTERNAL_START_POSITION, //(1.75, 0)
        RED_INTERNAL_START_POSITION, //(0.75, 0)
        BLUE_A, //
        BLUE_B, //
        BLUE_C, //
        RED_A, //
        RED_B, //
        RED_C, //
        BLUE_SHOOTING_POINT, // to the upper target
        RED_SHOOTING_POINT, // to the upper target
        BLUE_FIRST_STICK_POINT, // the left one
        BLUE_MIDDLE_STICK_POINT, // the middle
        BLUE_THIRD_STICK_POINT, //the right one
        RED_FIRST_STICK_POINT, // the right one
        RED_MIDDLE_STICK_POINT, // the middle
        RED_THIRD_STICK_POINT, //the left one
        BLUE_PARKING,
        RED_PARKING,
        START0_0


    }

    LOCATION name;
    public double x;
    public double y;

    public Location(LOCATION name, double x, double y) {
        this.name = name;
        this.x = x;
        this.y = y;
    }
}
