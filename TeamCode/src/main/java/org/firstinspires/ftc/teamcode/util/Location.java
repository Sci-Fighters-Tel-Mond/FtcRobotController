package org.firstinspires.ftc.teamcode.util;

public class Location {

    public double x;
    public double y;

    public Location(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Location offset(double offset) {
        return new Location(this.x,this.y + offset);
    }

}
