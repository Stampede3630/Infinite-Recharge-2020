package frc.robot;

/**
 * Defines a sequence of rumble outputs.
 * <p>
 * </p>
 * Use this instead of providing a string directly to
 * {@link RumbleSystem#setRumbleSequence(String)} for better performance, as
 * this class caches the compiled commands instead of recompiling them on each
 * call.
 * <p>
 * </p>
 * Using this class also allows the use of per-object cooldowns.
 * <p>
 * </p>
 * This class should be created during initialization and cached to be reused
 * for the same kind of rumble feedback, as cooldown works per-object (so two
 * identical sequence objects wouldn't respect their cooldown).
 */
public class RumbleSequence {
	/**
	 * Contains predefined sequence strings to be passed to the constructor.
	 */
	public static class Sequences {
		public static final String SINGLE_PING_LIGHT = "RR";
		public static final String SINGLE_PING_HEAVY = "LL";
		public static final String DOUBLE_PING = "RR----RR";
		public static final String DOUBLE_PING_WITH_HEAVY = "RBl--lBR";
		public static final String IMPERIAL_RUMBLE = ""
				+ "RRRRRRRR--------RRRRRRRR--------RRRRRRRR--------"
				+ "LLLLLLLLRRRR----RRRRRRRR--------LLLLLLLLRRRR----RRRRRRRR------------------------"
				+ "RRRRRRRR--------RRRRRRRR--------RRRRRRRR--------"
				+ "RRRRRR--RRR-----RRRRRRR---------LLLLLLLLRRRR----RRRRRRRR------------------------"
				+ "RRRRRRRR--------LLLLLLLL--------RRRRRRRR--------RRRRRRRR----RR--"
				+ "RR--RR--RRRR--------------LLLLLLRRRRRRRR--------RRRRRRRR----RR--"
				+ "RR--RR--RRRR--------------LLLLLLRRRRRRRR--------"
				+ "LLLLLLLL--RRRR--RRRRRRRR--------"
				+ "LLLLLLLL--RRRR--RRRRRRRR";
	}

	/**
	 * Array of compiled commands
	 */
	private RumbleSystem.RumbleCommand[] commands;

	/**
	 * Number of ticks during which the sequence can't be restarted. <br>
	 * Note that the sequence can still restart if {@link RumbleSystem#stop()} is
	 * called or another sequence starts.
	 */
	private int cooldown;

	/**
	 * Creates a RumbleSequence with no cooldown.
	 *
	 * @param stringSequence String sequence defining the sequence of rumble
	 *                       outputs.<br>
	 *                       <b>See key at {@link #RumbleSequence(String, int)}</b>.
	 */
	public RumbleSequence(String stringSequence) {
		commands = RumbleSystem.compileSequence(stringSequence);
		cooldown = 0;
	}

	/**
	 * Creates a RumbleSequence.
	 *
	 * @param stringSequence String sequence defining the sequence of rumble
	 *                       outputs.
	 *                       <p>
	 *                       </p>
	 *                       Key:<br>
	 *                       - (dash): Off<br>
	 *                       l (lowercase L): Weak left rumble<br>
	 *                       L (uppercase L): String left rumble<br>
	 *                       r (lowercase R): Weak right rumble<br>
	 *                       R (uppercase R): String right rumble<br>
	 *                       b (lowercase B): Weak left & right rumble<br>
	 *                       B (uppercase B): String left & right rumble
	 *                       <p>
	 *                       </p>
	 *                       Example: "LLll--RRrr"<br>
	 *                       Left strong rumble for 2 ticks, weak for 2 ticks, then
	 *                       off for 2 ticks. Repeated for the right side.
	 * @param cooldown       Cooldown until the sequence can be restarted.<br>
	 *                       Set to 0 to ignore.<br>
	 *                       Set to a high value (such as {@link Integer#MAX_VALUE})
	 *                       to disable repeats until {@link RumbleSystem#stop()} is
	 *                       called or another sequence is started.<br>
	 *                       See {@link #cooldown}.
	 */
	public RumbleSequence(String stringSequence, int cooldown) {
		commands = RumbleSystem.compileSequence(stringSequence);
		this.cooldown = cooldown;
	}

	/**
	 * @return Array of compiled rumble commands.
	 */
	public RumbleSystem.RumbleCommand[] getCommands() {
		return commands;
	}

	/**
	 * @return Sequence cooldown.
	 */
	public int getCooldown() {
		return cooldown;
	}

	/**
	 * Activates this RumbleSequence.
	 */
	public void trigger() {
		RumbleSystem.setRumbleSequence(this);
	}

	/**
	 * Stops this RumbleSequence (if active) and resets its cooldown.<br>
	 * Doesn't stop other currently active RumbleSequences.
	 */
	public void reset() {
		RumbleSystem.resetCooldown(this);
	}
}
