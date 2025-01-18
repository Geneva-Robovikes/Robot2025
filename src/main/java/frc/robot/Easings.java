package frc.robot;

public class Easings {
    public final double drive(double input) {
        //https://www.desmos.com/calculator/lnkcvttfsx

        double pi = Math.PI;
        input -= Math.sin(2 * pi * input) / (2 * pi);
        return input;
    }
}
