package frc.robot;

import java.util.ArrayDeque;
import java.util.Queue;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * System handling Rumble feedback.
 * <br>
 * {@link #update()} must be called periodically every tick.
 */
public class RumbleSystem
{
	/**
	 * Holds constants for the strength of rumble feedback.
	 */
	public static class RumbleStrength
	{
		private static final double OFF = 0;
		private static final double WEAK = 0.5;
		private static final double STRONG = 1;
	}

	/**
	 * Current command index (for {@link #commands}).
	 * Advances after every successful command.
	 */
	private static int currentCommandIndex;
	/**
	 * Cached sequence of {@link RumbleCommand RumbleCommand}s.
	 */
	private static RumbleCommand[] commands;
	/**
	 * Active {@link RumbleSequence}.
	 * Null if stopped, or a custom sequence is currently running (using {@link #setRumbleSequence(String)}).
	 */
	private static RumbleSequence activeRumbleSequence;

	/**
	 * Number of ticks elapsed. Used to track whether and when the next command should be executed.
	 */
	private static int ticksElapsed;
	/**
	 * The number of times {@link #update()} was called since the last tick increment.
	 */
	private static int updatesSinceLastTick;

	/**
	 * Determines how many updates need to run to advance by one tick.
	 * Essentially the timescale.
	 * <br>
	 * Value of 1 increments the tick every update.
	 */
	private static final int UPDATES_PER_TICK = 2;

	/**
	 * Main update cycle, call this periodically.
	 */
	public static void update()
	{
		updatesSinceLastTick++;
		if (updatesSinceLastTick >= UPDATES_PER_TICK)
		{
			updatesSinceLastTick = 0;
			if (commands != null && commands.length > currentCommandIndex)
			{
				RumbleCommand command = commands[currentCommandIndex];
				if (command.tick <= ticksElapsed)
				{
					command.process();
					currentCommandIndex++;
				}
			}
			ticksElapsed++;
		}
	}

	/**
	 * Set a custom rumble sequence using a string.
	 * <br>
	 * <b>Not recommended</b>, use {@link #setRumbleSequence(RumbleSequence)} instead for better performance.
	 *
	 * @param sequence Encoded String sequence. <br><b>See key at {@link RumbleSequence#RumbleSequence(String, int)}</b>.
	 */
	public static void setRumbleSequence(String sequence)
	{
		stop();
		commands = compileSequence(sequence);
	}

	/**
	 * Set a sequence using a pre-instantiated {@link RumbleSequence} object.
	 *
	 * @param sequence {@link RumbleSequence} object.<br>
	 *                 Calling this method repeatedly with the same {@link RumbleSequence} object respects cooldowns (i.e. the sequence isn't restarted every call if it has a cooldown).
	 */
	public static void setRumbleSequence(RumbleSequence sequence)
	{
		if (sequence == null)
		{
			stop();
			return;
		}
		// If the set sequence is the same as the currently active one (i.e. restart)
		// Ignore if the cooldown hasn't yet passed
		if (activeRumbleSequence == sequence && sequence.getCooldown() > ticksElapsed) return;
		stop();
		commands = sequence.getCommands();
		activeRumbleSequence = sequence;
	}

	/**
	 * Compiles a String representation of a rumble sequence into an array of {@link RumbleCommand}s.
	 *
	 * @param sequence String representation of the rumble sequence. <br><b>See key at {@link RumbleSequence#RumbleSequence(String, int)}</b>.
	 * @return Compiled array of {@link RumbleCommand}s.
	 */
	public static RumbleCommand[] compileSequence(String sequence)
	{
		Queue<RumbleCommand> queue = new ArrayDeque<>();
		char lastChar = ' ';
		char[] charArray = sequence.toCharArray();
		int i = 0;
		for (int charArrayLength = charArray.length; i < charArrayLength; i++)
		{
			char c = charArray[i];
			if (lastChar != c)
			{
				switch (c)
				{
					case '-':
						queue.add(new RumbleCommand(i, RumbleStrength.OFF, RumbleStrength.OFF));
						break;
					case 'l':
						queue.add(new RumbleCommand(i, RumbleStrength.WEAK, RumbleStrength.OFF));
						break;
					case 'L':
						queue.add(new RumbleCommand(i, RumbleStrength.STRONG, RumbleStrength.OFF));
						break;
					case 'r':
						queue.add(new RumbleCommand(i, RumbleStrength.OFF, RumbleStrength.WEAK));
						break;
					case 'R':
						queue.add(new RumbleCommand(i, RumbleStrength.OFF, RumbleStrength.STRONG));
						break;
					case 'b':
						queue.add(new RumbleCommand(i, RumbleStrength.WEAK, RumbleStrength.WEAK));
						break;
					case 'B':
						queue.add(new RumbleCommand(i, RumbleStrength.STRONG, RumbleStrength.STRONG));
						break;
				}
			}
			lastChar = c;
		}
		// Add one Off command at the end
		queue.add(new RumbleCommand(i, RumbleStrength.OFF, RumbleStrength.OFF));
		RumbleCommand[] commandArray = new RumbleCommand[queue.size()];
		queue.toArray(commandArray);
		return commandArray;
	}

	/**
	 * Stops all rumble output and resets the state.<br>
	 * This also resets all cooldowns.
	 */
	public static void stop()
	{
		commands = null;
		ticksElapsed = 0;
		updatesSinceLastTick = 0;
		currentCommandIndex = 0;
		activeRumbleSequence = null;
		Robot.controller.setRumble(RumbleType.kLeftRumble, 0);
		Robot.controller.setRumble(RumbleType.kRightRumble, 0);
	}

	/**
	 * Stops if the provided sequence is the currently active one, resetting its cooldown.<br>
	 * Otherwise continues running the current sequence
	 * @param sequence The sequence to have its cooldown reset.
	 */
	public static void resetCooldown(RumbleSequence sequence)
	{
		if (sequence == null || activeRumbleSequence != sequence) return;
		stop();
	}

	/**
	 * Defines the states of both rumble outputs at a given tick.<br>
	 * Used when compiling a String representation of a rumble sequence.
	 */
	public static class RumbleCommand
	{
		/**
		 * Tick at which the command is scheduled to trigger.
		 */
		private int tick;
		/**
		 * Left rumble intensity. Should be within range 0-1.
		 */
		private double leftRumble;
		/**
		 * Right rumble intensity. Should be within range 0-1.
		 */
		private double rightRumble;

		/**
		 * @return See {@link #tick}.
		 */
		public int getTick()
		{
			return tick;
		}

		/**
		 * @return See {@link #leftRumble}.
		 */
		public double getLeftRumble()
		{
			return leftRumble;
		}

		/**
		 * @return See {@link #rightRumble}.
		 */
		public double getRightRumble()
		{
			return rightRumble;
		}

		/**
		 * Defines a Rumble Command.
		 *
		 * @param tick        Tick at which the command is scheduled to execute.
		 * @param leftRumble  Left rumble intensity.
		 * @param rightRumble Right rumble intensity.
		 */
		public RumbleCommand(int tick, double leftRumble, double rightRumble)
		{
			this.tick = tick;
			this.leftRumble = leftRumble;
			this.rightRumble = rightRumble;
		}

		/**
		 * Process the command.
		 */
		public void process()
		{
			Robot.controller.setRumble(RumbleType.kLeftRumble, leftRumble);
			Robot.controller.setRumble(RumbleType.kRightRumble, rightRumble);
		}
	}
}
