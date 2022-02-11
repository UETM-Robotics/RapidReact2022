package frc.robot.Actions.Framework;

import java.util.LinkedHashSet;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Actions.Autonomy.AutomatedAction;
import frc.robot.subsystems.Subsystem;

public class TeleopActionRunner {
    
    private static LinkedHashSet<AutomatedAction> mActionList = new LinkedHashSet<>();

	public static void processActions() {
		try {
			if (mActionList.size() > 0) {
				mActionList.forEach((action) -> {
					if (!action.isStarted())
						action.start();
					action.update();
				});
				mActionList.removeIf((action) -> {
					boolean finished = false;
                    System.out.println(action.getClass().getSimpleName() + " Action Running!");
					if (action.isFinished()) {
						finished = true;
						action.done();
					}
					return finished;
				});
			}
		} catch (Exception ex) {
            DriverStation.reportError("Fatal Error in Teleop Action Runner (process actions)", true);
		}
	}

	public static boolean runAction(AutomatedAction action) {
		try {
			if (mActionList.size() > 0) {
				mActionList.removeIf((xAction) -> {
					for (Subsystem xSubsystem : xAction.getRequiredSubsystems()) {
						if (action.getRequiredSubsystems().contains(xSubsystem)) {
							xAction.purgeActions();
							return true;
						}
					}
					return false;
				});
			}
			mActionList.add(action);
		} catch (Exception ex) {
            DriverStation.reportError("Fatal Error in Teleop Action Runner (run action)", true);
			return false;
		}

		return true;
	}
}
