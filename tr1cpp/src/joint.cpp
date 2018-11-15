#include <stdlib.h>
#include <math.h>
#include <stdexcept>
#include "ros/ros.h"
#include <tr1cpp/joint.h>
#include <tr1cpp/serial.h>

#define PI 3.14159265359
#define TAU 6.28318530718

namespace tr1cpp
{
	Joint::Joint()
	{
		char serialName[13] = "/dev/ttyACM0";
		this->_serialPort = new SerialPort(serialName);
	}

	Joint::Joint(uint8_t motorId)
	{
		setMotorId(motorId);
	}

	Joint::~Joint()
	{

	}

	void Joint::setActuatorType(uint8_t actuatorType)
	{
		this->_actuatorType = actuatorType;
	}

	uint8_t Joint::getMotorId()
	{
		return this->_motorId;
	}

	void Joint::setMotorId(uint8_t motorId)
	{
		this->_motorId = motorId;
	}

	double Joint::_filterAngle(double angle)
	{
		_angleReads = _angleReads + 1;

		// put value at front of array
		for (int i = _filterPrevious - 1; i > 0; i--) {
			_previousAngles[i] = _previousAngles[i - 1];
		}
		_previousAngles[0] = angle;


		int filterIterations = _filterPrevious;
		if (_angleReads < _filterPrevious) {
			filterIterations = _angleReads;
		}

		double angleSum = 0;
		for (int i = 0; i < filterIterations; i++) {
			angleSum = angleSum + _previousAngles[i];
		}

		double filterResult = angleSum / (filterIterations * 1.0);

		//ROS_INFO("%f, %f, %f, %i", angle, angleSum, filterResult, filterIterations);

		return filterResult;
	}

	double Joint::readAngle()
	{
		if (name == "JointTorsoExtension") {
/*			uint16_t position;
			I2C i2cSlave = I2C(0, _getSlaveAddress());
			uint8_t result = i2cSlave.readBytes(_motorId, 4, position);
			if (result == 1) {
				double p = position * readRatio;
				return p;
			}*/
			return 0;
		} else if (_actuatorType == ACTUATOR_TYPE_MOTOR) {
			if (_serialPort->isConnected()) {
				int res_size = 32;
				char req[255];
				char res[res_size];
				snprintf(req, 255, "1 0x%x %i ;", _getSlaveAddress(), _motorId);
				bool success = _serialPort->readSerialPort(req, res);
				if (!success) {
					ROS_WARN("Could not write to serial port");
				} else {
					int motorId;
					int position;
					int variableIndex = 0;
					int bufferIndex = 0;
					char buf[res_size];
					for (int i = 0; i < res_size; i++) {
						char c = res[i];
						if (c == ' ') {
							buf[bufferIndex + 1] = '\0';
							if (variableIndex == 0) {
								// i2c address, not needed
							} else if (variableIndex == 1) {
								motorId = atoi(buf);
							} else if (variableIndex == 2) {
								position = atoi(buf);
							}
							
							variableIndex++;
							bufferIndex = 0;
							memset(buf, 0, res_size);
						} else if (c == ';') {
							break;
						} else {
							buf[bufferIndex] = c;
							bufferIndex++;
						}
					}
					
					if (motorId == this->_motorId) {
						double angle = (position / sensorResolution * TAU);
						angle = _filterAngle(angle);
						angle += angleOffset;
						if (angle > PI) angle -= TAU;
						if (angle < -PI) angle += TAU;
						angle *= readRatio;

						// probably data noise, skip and return previous read
						if (abs(angle - this->_previousRead) > PI / 4.0 && this->_noiseCount < 3) {
							this->_noiseCount++;
							return this->_previousRead;
						}

						this->_noiseCount = 0;
						this->_previousRead = angle;
						return angle;
					} else {
						return this->_previousRead;
					}
				}
			} else {
				ROS_WARN("Serial port is not connected");
			}
			return 0;
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			return _previousEffort;
		}
		else
		{
			return 0;
		}
	}

	void Joint::actuate(double effort, uint8_t duration = 30)
	{
		if (_actuatorType == ACTUATOR_TYPE_MOTOR)
		{
			if (effort > 1.0) effort = 1.0;
			if (effort < -1.0) effort = -1.0;
			if (abs(effort * 100.0) < 20) return; // because it's too little to do anything

			uint8_t data[4];
			data[3] = duration;
			if (_serialPort->isConnected()) {
				if (effort > 1.0) effort = 1.0;
				if (effort < -1.0) effort = -1.0;
				uint8_t speed = floor(abs(effort * 100));
				uint8_t direction = (effort > 0);

				char data[255];
				snprintf(data, 255, "0 0x%x %i %i %i %i ;", _getSlaveAddress(), _motorId, speed, direction, duration);
				bool success = _serialPort->writeSerialPort(data);
				if (!success) {
					ROS_WARN("Could not write to serial port");
				}
			} else {
				ROS_WARN("Serial port is not connected");
			}
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			if (floor(effort * 100.0) != floor(this->_previousEffort * 100.0)) {
				if (_serialPort->isConnected()) {
					double magnitude = effort * 100.0;
					uint8_t servoValue = floor(_minServoValue + ((_maxServoValue - _minServoValue) * (magnitude / 100.0)));

					char data[255];
					snprintf(data, 255, "0 0x%x %i %i 0 0 ;", _getSlaveAddress(), _motorId, servoValue);
					bool success = _serialPort->writeSerialPort(data);
					if (!success) {
						ROS_WARN("Could not write to serial port");
					}
				} else {
					ROS_WARN("Serial port is not connected");
				}
			}
		}

		this->_previousEffort = effort;
	}

	uint8_t Joint::_getSlaveAddress()
	{
		if (_motorId > 0 && _motorId <= 8)
		{
			return ARM_RIGHT_SLAVE_ADDRESS;
		}
		else if (_motorId > 0 && _motorId <= 13)
		{
			return BASE_SLAVE_ADDRESS;
		}
		else if (_motorId > 0 && _motorId <= 15)
		{
			return HEAD_SLAVE_ADDRESS;
		}
		else if (_motorId > 0 && _motorId <= 23)
		{
			return ARM_LEFT_SLAVE_ADDRESS;
		}
		else
		{
			ROS_ERROR("Invalid MotorID: %i", _motorId);
			return -1;
		}
	}

	void Joint::setServoLimits(uint8_t minValue, uint8_t maxValue)
	{
		this->_minServoValue = minValue;
		this->_maxServoValue = maxValue;
	}

	double Joint::getPreviousEffort() {
		return this->_previousEffort;
	}

	void Joint::_prepareI2CWrite(uint8_t result[4], double effort)
	{
		if (_actuatorType == ACTUATOR_TYPE_MOTOR)
		{
			if (effort > 1.0) effort = 1.0;
			if (effort < -1.0) effort = -1.0;
			uint8_t speed = floor(abs(effort * 100));
			uint8_t direction = (effort > 0);
			//uint8_t duration = 5;

			result[0] = _motorId;
			result[1] = speed;
			result[2] = direction;
			//result[3] = duration;
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			/*if (name != "JointRightGripper") {
				effort = (effort + 1.5708) / 3.1415;
				if (effort > 1.0) effort = 1.0;
				if (effort < 0.0) effort = 0.0;
			}*/

			double magnitude = effort * 100.0;
			uint8_t servoValue = floor(_minServoValue + ((_maxServoValue - _minServoValue) * (magnitude / 100.0)));

			result[0] = _motorId;
			result[1] = servoValue;
			result[2] = 0;
			result[3] = 0;

			//ROS_INFO("name: %s, minServoValue: %i, maxServoValue: %i, effort: %f, magnitude: %f, servoValue: %i", name.c_str(), _minServoValue, _maxServoValue, effort, magnitude, servoValue);
		}
	}
	
	int Joint::getActuatorType()
	{
		return _actuatorType;
	}
}
