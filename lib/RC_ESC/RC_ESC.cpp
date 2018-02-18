
#include "RC_ESC.h"
#include <Arduino.h>


//https://hackaday.com/2016/10/31/whats-new-esp-32-testing-the-arduino-esp32-library/
RC_ESC::RC_ESC(int channel, int pinpwm, bool reversed)
{
	RC_ESC(channel, pinpwm, reversed, 41, 212);
}
RC_ESC::RC_ESC(int channel, int pinpwm, bool reversed, int min, int max)
{
	_pinPWM = pinpwm;
	_reverse = reversed;
	_channel = channel;

	_min = min;
	_max = max;

	pinMode(_pinPWM, OUTPUT);

	//doing this because it seems to initialize the timer correclty... TODO: More research in to why?
	//ledcSetup(5, 20000, 8);

	ledcSetup(_channel, 50, 16);
	ledcAttachPin(_pinPWM, _channel);
	ledcWrite(_channel, 4924);
}

void RC_ESC::SetMotorSpeed(float speed)
{
	if (_reverse)
		speed = speed * -1;

	if (speed > 0)
	{
		ledcWrite(_channel, map(speed, 0, 255, 4924, 6565));
	}
	else
	{
		ledcWrite(_channel, map(speed, -255, 0, 3283, 4924));
	}
}
