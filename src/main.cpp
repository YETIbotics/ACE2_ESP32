#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <PS4BT.h>
#include <usbhub.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <CytronMD10.h>
#include <RC_ESC.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

WiFiMulti wifiMulti;

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
//PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
PS4BT PS4(&Btd);

//CytronMD10 LIFT(0, 27, 14, false);
CytronMD10 DriveRight(7, 32, 33, false);
CytronMD10 DriveLeft(6, 25, 26, true);
RC_ESC Lift(0, 27, true);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void timerLoop();
void WriteRobot();
void Task();
void ReadController();
void SetupOTA();

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
	Serial.begin(115200);

	//pinMode(14, INPUT_PULLUP);
	//pinMode(12, INPUT_PULLUP);
	//pinMode(15, INPUT_PULLUP);

	LeftJoystickY = 0.0;
	LeftJoystickX = 0.0;
	RightJoystickY = 0.0;
	RightJoystickX = 0.0;
	TriggerAggregate = 0.0;

	DriveRightSpeed = 0.0;
	DriveLeftSpeed = 0.0;
	MOGOSpeed = 0.0;

	isArcadeDrive = false;

	if (Usb.Init() == -1)
	{

		while (1)
			; //halt
	}

	wifiMulti.addAP("RoboticHuskies", "robotsrule");
	wifiMulti.addAP("NBS", "N3metr12");
	wifiMulti.addAP("FreePublicWIFI", "");
	wifiMulti.run();

	// Serial.println("Connecting Wifi...");
	// if(wifiMulti.run() == WL_CONNECTED) {
	//     Serial.println("");
	//     Serial.println("WiFi connected");
	//     Serial.println("IP address: ");
	//     Serial.println(WiFi.localIP());
	// }

	//t.setInterval(20, timerLoop);

	SetupOTA();

	if (!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while (1)
			;
	}

	delay(1000);

	bno.setExtCrystalUse(true);

	pinMode(2, OUTPUT);
	digitalWrite(2, HIGH);
}

unsigned long lastRun20 = millis();
unsigned long lastRun100 = millis();

void loop()
{
	ArduinoOTA.handle();
	Usb.Task();
	//t.run();
	if (millis() - lastRun20 >= 20)
	{
		timerLoop();

		//Serial.print(digitalRead(2));
		//Serial.print(":2 \t");
		//Serial.print(digitalRead(14));
		//Serial.print(":14 \t");
		//Serial.print(digitalRead(12));
		//Serial.print(":12 \t");
		//Serial.print(digitalRead(15));
		//Serial.print(":15 \t");
		//Serial.print(DriveRightSpeed);
		//Serial.print(":RtSpd \t");
		//Serial.print(DriveLeftSpeed);
		//Serial.print(":LtSpd \t");
		//Serial.print(MOGOSpeed);
		//Serial.print(":MGSpd \t");

		//Serial.println("");

		lastRun20 = millis();
	}

	if (millis() - lastRun100 >= 100)
	{
		sensors_event_t event;
		bno.getEvent(&event);

		/* Display the floating point data */
		Serial.print("X: ");
		Serial.print(event.orientation.x, 4);
		Serial.print("\tY: ");
		Serial.print(event.orientation.y, 4);
		Serial.print("\tZ: ");
		Serial.print(event.orientation.z, 4);
		Serial.println("");

		lastRun100 = millis();
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
	Lift.SetMotorSpeed(MOGOSpeed);

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

	if (true)
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

	if (PS4.connected())
	{

		//L2 Trigger
		if (PS4.getAnalogButton(R2))
		{
			TriggerAggregate = PS4.getAnalogButton(R2) * 1;
		}
		//R2 Trigger
		else if (PS4.getAnalogButton(L2))
		{
			TriggerAggregate = PS4.getAnalogButton(L2) * -1;
		}

		if (PS4.getAnalogHat(LeftHatY) > 137)
		{
			//Going backwards
			LeftJoystickY = map(PS4.getAnalogHat(LeftHatY), 137, 255, -155, -255);
		}
		else if (PS4.getAnalogHat(LeftHatY) < 117)
		{
			//going forwards
			LeftJoystickY = map(PS4.getAnalogHat(LeftHatY), 117, 0, 155, 255);
		}
		else
		{
			LeftJoystickY = 0;
		}

		if (PS4.getAnalogHat(RightHatY) > 137)
		{
			RightJoystickY = map(PS4.getAnalogHat(RightHatY), 137, 255, -155, -255);
		}
		else if (PS4.getAnalogHat(RightHatY) < 117)
		{
			RightJoystickY = map(PS4.getAnalogHat(RightHatY), 117, 0, 155, 255);
		}
		else
		{
			RightJoystickY = 0;
		}

		if (PS4.getButtonClick(X))
		{
			isArcadeDrive = !isArcadeDrive;
		}

		if (PS4.getButtonClick(PS))
		{
			PS4.disconnect();
		}
	}
}

void SetupOTA()
{

	// Port defaults to 3232
	// ArduinoOTA.setPort(3232);

	// Hostname defaults to esp3232-[MAC]
	ArduinoOTA.setHostname("ace2");

	// No authentication by default
	// ArduinoOTA.setPassword("admin");

	// Password can be set with it's md5 value as well
	// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

	ArduinoOTA
		.onStart([]() {

			String type;
			if (ArduinoOTA.getCommand() == U_FLASH)
				type = "sketch";
			else // U_SPIFFS
				type = "filesystem";

			// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
			Serial.println("Start updating " + type);
		})
		.onEnd([]() {
			Serial.println("\nEnd");
		})
		.onProgress([](unsigned int progress, unsigned int total) {

			Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
		})
		.onError([](ota_error_t error) {
			Serial.printf("Error[%u]: ", error);
			if (error == OTA_AUTH_ERROR)
				Serial.println("Auth Failed");
			else if (error == OTA_BEGIN_ERROR)
				Serial.println("Begin Failed");
			else if (error == OTA_CONNECT_ERROR)
				Serial.println("Connect Failed");
			else if (error == OTA_RECEIVE_ERROR)
				Serial.println("Receive Failed");
			else if (error == OTA_END_ERROR)
				Serial.println("End Failed");
		});

	ArduinoOTA.begin();
}