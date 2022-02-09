package frc.robot.Utilities;

import java.util.*;

import frc.robot.Actions.AutomatedAction;
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
					if (action.isFinished()) {
						finished = true;
						action.done();
					}
					return finished;
				});
			}
		} catch (Exception ex) {
            System.out.println("Error Running TeleOp Command");
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
			System.out.println("Error In Running Teleop Action");
			return false;
		}

		return true;
	}
}