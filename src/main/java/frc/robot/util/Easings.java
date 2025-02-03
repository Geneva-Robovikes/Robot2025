package frc.robot.util;

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

    public final double deadzonePositiveFirst(double input, double deadzone) {
        double modifiedDeadzone = deadzone / (1 - deadzone);
        input = (1 + modifiedDeadzone) * input - modifiedDeadzone;
        return input;
    }
    public final double deadzonePositiveLast(double input, double minimum) {
        input = input * (1 - minimum) + minimum;
        return input;
    }
    public final double deadzoneNegativeFirst(double input, double deadzone) {
        double modifiedDeadzone = deadzone / (1 - deadzone);
        input = (1 + modifiedDeadzone) * input + modifiedDeadzone;
        return input;
    }
    public final double deadzoneNegativeLast(double input, double minimum) {
        input = input * (1 - minimum) - minimum;
        return input;
    }
}
