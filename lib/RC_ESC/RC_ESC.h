#pragma once

class RC_ESC{
	
public:

	RC_ESC(int channel, int pinpwm, bool reversed);
	RC_ESC(int channel, int pinpwm, bool reversed, int min, int max);
	void SetMotorSpeed(float speed);

private:

	int _pinPWM;
	bool _reverse;
	int _channel;
	int _min;
	int _max;
};


