package frc.robot.Utilities;

import frc.robot.Loops.Looper;

public interface CustomSubsystem {
	void init();
	void subsystemHome();
	void registerEnabledLoops(Looper in);
}
