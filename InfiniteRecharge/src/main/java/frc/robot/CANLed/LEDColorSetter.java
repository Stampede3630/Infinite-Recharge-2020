package frc.robot.CANLed;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.CANifier;
import frc.robot.RobotMap;

public class LEDColorSetter implements ILoopable
{

    public float Hue;
	public float Saturation;
	public float Value;
    private static float _rgb[] = new float[3];

    RobotMap robotMap = RobotMap.getRobotMap();
    
    private MovingAverage _averageR = new MovingAverage(1); //make it so its not immediate color change - 2 maybe?
	private MovingAverage _averageG = new MovingAverage(1);
	private MovingAverage _averageB = new MovingAverage(1);

	/* ILoopable */
	public void onStart() { }

	public void onStop() { }

	public boolean isDone() { return false; }

	public void onLoop() {

		if (Saturation > 1) {
			Saturation = 1;
		}
		if (Saturation < 0)
			Saturation = 0;

		if (Value > 1)
			Value = 1;
		if (Value < 0)
			Value = 0;

		/* Convert to HSV to RGB */
		_rgb = HsvToRgb.convert(Hue, Saturation, Value);

		_rgb[0] = _averageR.process(_rgb[0]);
		_rgb[1] = _averageG.process(_rgb[1]);
		_rgb[2] = _averageB.process(_rgb[2]);

        /* Update CANifier's LED strip */
		robotMap.canifier.setLEDOutput(_rgb[0], CANifier.LEDChannel.LEDChannelA);
		robotMap.canifier.setLEDOutput(_rgb[1], CANifier.LEDChannel.LEDChannelB);
		robotMap.canifier.setLEDOutput(_rgb[2], CANifier.LEDChannel.LEDChannelC);
	}

}