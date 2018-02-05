#include <XBOXRECV.h>
#include <CytronMD10.h>

USB Usb;
XBOXRECV Xbox(&Usb);

CytronMD10 DriveLeft(0, 27, 14, false);
CytronMD10 DriveRight(1, 32, 33, false);
CytronMD10 ForkLift(2, 25, 26, false);

void timerLoop();
void WriteRobot();
void Task();
void ReadController();

float LeftJoystickY;
float LeftJoystickX;
float RightJoystickY;
float RightJoystickX;
float TriggerAggregate;

float DriveRightSpeed;
float DriveLeftSpeed;
float MOGOSpeed;

float isArcadeDrive;

void setup()
{

	LeftJoystickY = 0.0;
	LeftJoystickX = 0.0;
	RightJoystickY = 0.0;
	RightJoystickX = 0.0;
	TriggerAggregate = 0.0;

	DriveRightSpeed = 0.0;
	DriveLeftSpeed = 0.0;
	MOGOSpeed = 0.0;

	isArcadeDrive = false;

	Serial.begin(115200);

	if (Usb.Init() == -1)
	{
		Serial.print(F("\r\nOSC did not start"));
		while (1)
			; //halt
	}
	Serial.print(F("\r\nXbox Wireless Receiver Library Started"));

	// wifiMulti.addAP("RoboticHuskies", "robotsrule");
	// wifiMulti.addAP("NBS", "N3metr12");
	// wifiMulti.addAP("FreePublicWIFI", "");

	// Serial.println("Connecting Wifi...");
	// if(wifiMulti.run() == WL_CONNECTED) {
	//     Serial.println("");
	//     Serial.println("WiFi connected");
	//     Serial.println("IP address: ");
	//     Serial.println(WiFi.localIP());
	// }

	//t.setInterval(20, timerLoop);
}

unsigned long lastRun = millis();
void loop()
{
	Usb.Task();
	//t.run();
	if (millis() - lastRun >= 20)
	{
		timerLoop();
	}
}

void timerLoop()
{
	ReadController();
	Task();
	WriteRobot();
}

void WriteRobot()
{
	DriveRight.SetMotorSpeed(DriveRightSpeed);
	DriveLeft.SetMotorSpeed(DriveLeftSpeed);
	ForkLift.SetMotorSpeed(MOGOSpeed);

	//Serial.println(DriveLeftSpeed);
}

void Task()
{

	if (!isArcadeDrive)
	{
		DriveRightSpeed = RightJoystickY;
		DriveLeftSpeed = LeftJoystickY;
	}
	else
	{
		DriveLeftSpeed = (RightJoystickY + LeftJoystickY) + (RightJoystickX + LeftJoystickX);
		DriveRightSpeed = (RightJoystickY + LeftJoystickY) - (RightJoystickX + LeftJoystickX);

		if (DriveLeftSpeed > 255)
			DriveLeftSpeed = 255;

		if (DriveLeftSpeed < -255)
			DriveLeftSpeed = -255;

		if (DriveRightSpeed > 255)
			DriveRightSpeed = 255;

		if (DriveRightSpeed < -255)
			DriveRightSpeed = -255;
	}

	MOGOSpeed = TriggerAggregate;

	if (false)
	{
		//do exponential

		DriveRightSpeed = map((DriveRightSpeed * abs(DriveRightSpeed)), -65025, 65025, -255, 255);
		DriveLeftSpeed = map((DriveLeftSpeed * abs(DriveLeftSpeed)), -65025, 65025, -255, 255);
	}

	if (false)
	{
		int maxDiff = 60;
		if (DriveLeftSpeed > 0 && DriveRightSpeed > 0)
		{
			if (abs(DriveLeftSpeed - DriveRightSpeed) > maxDiff)
			{
				if (DriveLeftSpeed > DriveRightSpeed)
				{
					DriveRightSpeed = DriveLeftSpeed - maxDiff;
				}
				else
				{
					DriveLeftSpeed = DriveRightSpeed - maxDiff;
				}
			}
		}
		else if (DriveLeftSpeed < 0 && DriveRightSpeed < 0)
		{
			if (abs(DriveLeftSpeed - DriveRightSpeed) > maxDiff)
			{
				if (DriveLeftSpeed < DriveRightSpeed)
				{
					DriveRightSpeed = DriveLeftSpeed - maxDiff;
				}
				else
				{
					DriveLeftSpeed = DriveRightSpeed - maxDiff;
				}
			}
		}
	}

	//DriveLeftSpeed = -255;
	//DriveRightSpeed = -255;
	//MOGOSpeed = -255;
}

void ReadController()
{
	LeftJoystickY = 0.0;
	LeftJoystickX = 0.0;
	RightJoystickY = 0.0;
	RightJoystickX = 0.0;
	TriggerAggregate = 0.0;

	if (Xbox.XboxReceiverConnected)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			if (Xbox.Xbox360Connected[i])
			{
				//L2 Trigger
				if (Xbox.getButtonPress(R2, i))
				{
					TriggerAggregate = 255.0 / 255 * Xbox.getButtonPress(R2, i) * 1;
				}
				//R2 Trigger
				else if (Xbox.getButtonPress(L2, i))
				{
					TriggerAggregate = 255.0 / 255 * Xbox.getButtonPress(L2, i) * -1;
				}

				if (Xbox.getAnalogHat(LeftHatX, i) > 7500 || Xbox.getAnalogHat(LeftHatX, i) < -7500 || Xbox.getAnalogHat(LeftHatY, i) > 7500 || Xbox.getAnalogHat(LeftHatY, i) < -7500 || Xbox.getAnalogHat(RightHatX, i) > 7500 || Xbox.getAnalogHat(RightHatX, i) < -7500 || Xbox.getAnalogHat(RightHatY, i) > 7500 || Xbox.getAnalogHat(RightHatY, i) < -7500)
				{
					if (Xbox.getAnalogHat(LeftHatX, i) > 7500 || Xbox.getAnalogHat(LeftHatX, i) < -7500)
					{
						LeftJoystickX = map(Xbox.getAnalogHat(LeftHatX, i), -32767, 32767, -255, 255); //255.0 / 32767 * Xbox.getAnalogHat(LeftHatX, i);
					}
					if (Xbox.getAnalogHat(LeftHatY, i) > 7500 || Xbox.getAnalogHat(LeftHatY, i) < -7500)
					{
						LeftJoystickY = map(Xbox.getAnalogHat(LeftHatY, i), -32767, 32767, -255, 255); //255.0 / 32767 * Xbox.getAnalogHat(LeftHatY, i);
					}
					if (Xbox.getAnalogHat(RightHatX, i) > 7500 || Xbox.getAnalogHat(RightHatX, i) < -7500)
					{
						RightJoystickX = map(Xbox.getAnalogHat(RightHatX, i), -32767, 32767, -255, 255); //255.0 / 32767 * Xbox.getAnalogHat(RightHatX, i);
					}
					if (Xbox.getAnalogHat(RightHatY, i) > 7500 || Xbox.getAnalogHat(RightHatY, i) < -7500)
					{
						RightJoystickY = map(Xbox.getAnalogHat(RightHatY, i), -32767, 32767, -255, 255); //255.0 / 32767 * ;
					}
				}

				if (Xbox.getButtonPress(UP, i))
				{
					LeftJoystickY = 200;
					RightJoystickY = 200;
				}

				if (Xbox.getButtonClick(START, i))
				{
					isArcadeDrive = !isArcadeDrive;
				}

				if (Xbox.getButtonClick(XBOX, i))
				{

					DriveRight.SetMotorSpeed(255); //rt
					DriveLeft.SetMotorSpeed(255);  //lt
					delay(100);

					DriveRight.SetMotorSpeed(0); //rt
					DriveLeft.SetMotorSpeed(0);  //lt
					delay(2000);

					DriveRight.SetMotorSpeed(255); //rt
					DriveLeft.SetMotorSpeed(255);  //lt
					delay(1000);
				}
			}
		}
	}
}