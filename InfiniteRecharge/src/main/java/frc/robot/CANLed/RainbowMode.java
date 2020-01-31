package frc.robot.CANLed;

import com.ctre.phoenix.ILoopable;

public class RainbowMode implements ILoopable
{
    private float _hue;

	/* ILoopable */
    public void onStart() { }
    
	public void onStop() { }
    
    public boolean isDone() { return false; }
    
    public void onLoop() {
		/* Ramp through the outer rim of the HSV color wheel */
		_hue += 1;
		if (_hue >= 360) {
			_hue = 0;
		}

		/* Update LEDStrip/HSV target */
		TaskList.myLEDColorSetter.Hue = _hue;
		TaskList.myLEDColorSetter.Saturation = 1.0f;    // Outer rim of HSV color wheel
        TaskList.myLEDColorSetter.Value = 0.05f;        // Hard-code the brightness
        
        //System.out.println("Pretty rainbow colors :)");
	}

	public String toString() {
		return "AnimateLEDStrip:" + _hue;
	}
}