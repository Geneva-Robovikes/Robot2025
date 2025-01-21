package frc.robot;

public class Easings {
    public final double joystick(double input) {
        //https://www.desmos.com/calculator/lnkcvttfsx

        double pi = Math.PI;
        input -= Math.sin(2 * pi * input) / (2 * pi);
        return input;
    }

    public final double joystick(double input, double mod) {
        //https://www.desmos.com/calculator/biemsypgqq

        double pi = Math.PI;
        input -= Math.sin(2 * pi * input) / (2 * pi * mod);
        return input;
    }
}
