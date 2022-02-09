package frc.robot.Actions;

import java.util.Arrays;
import java.util.HashSet;

import frc.robot.Actions.Framework.Action;
import frc.robot.Utilities.TimeoutTimerActions;
import frc.robot.subsystems.Subsystem;

public class AutomatedAction implements Action {

	private HashSet<Subsystem> requiredSubsystems = new HashSet<>();
	private final TimeoutTimerActions mTimeoutTimer;
	private double mTimeout;
	private SeriesAction mAction;
	private boolean mStarted = false;

	public static AutomatedAction fromAction(Action action, double timeout, Subsystem... requirements) {
		AutomatedAction a = new AutomatedAction(action, timeout);
		a.addRequirements(requirements);
		return a;
	}

	public static AutomatedAction fromAction(SeriesAction action, double timeout, Subsystem... requirements) {
		AutomatedAction a = new AutomatedAction(action, timeout);
		a.addRequirements(requirements);
		return a;
	}

	public AutomatedAction(Action action, double timeout) {
		this(new SeriesAction(action), timeout);
	}

	public AutomatedAction(SeriesAction action, double timeout) {
		mAction = action;
		mTimeoutTimer = new TimeoutTimerActions(timeout);
		mTimeout = timeout;
	}

	public void addRequirements(Subsystem... subsystems) {
		requiredSubsystems.addAll(Arrays.asList(subsystems));
	}

	public HashSet<Subsystem> getRequiredSubsystems() {
		return requiredSubsystems;
	}

	public boolean isStarted() {
		return mStarted;
	}

	public double getTimeout() { return mTimeout; }

	public void purgeActions() {
		mAction.purgeActions();
	}

	@Override
	public boolean isFinished() {
		return (mTimeout > 0 && mTimeoutTimer.isTimedOut()) || mAction.isFinished();
	}

	@Override
	public void update() {
		mAction.update();
	}

	@Override
	public void done() {
		mAction.done();
	}

	@Override
	public void start() {
		mAction.start();
		mStarted = true;
	}
}