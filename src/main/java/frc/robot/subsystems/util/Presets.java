package frc.robot.subsystems.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.MotorSubsystem;

@SuppressWarnings("unused")
public final class Presets {
    public class LevelTwoPreset extends Command {
        private final MotorSubsystem motorSubsystem;

        public LevelTwoPreset(MotorSubsystem motorSubsystem) {
            this.motorSubsystem = motorSubsystem;
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {return false;}
    }

    public class LevelThreePreset extends Command {
        private final MotorSubsystem motorSubsystem;

        public LevelThreePreset(MotorSubsystem motorSubsystem) {
            this.motorSubsystem = motorSubsystem;
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {return false;}
    }
}
