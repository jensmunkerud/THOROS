#include "Logger.h"

#include "Motor.h"

Logger::Logger(Drone& droneState, Telemetry& telemetryState, Motor& motorState) :
drone{droneState},
telemetry{telemetryState},
motor{motorState},
ready{false},
logging{false},
includeFullTelemetry{false},
logStartMs{0},
lastWriteMs{0},
logPath{0},
hspi(HSPI)
{}

bool Logger::begin() {
	hspi.begin(LOGGER_SCK, LOGGER_MISO, LOGGER_MOSI, LOGGER_CS);
	ready = SD.begin(LOGGER_CS, hspi);
	if (ready && !SD.exists("/logs")) {
		SD.mkdir("/logs");
	}
	{
		DroneLockGuard droneLock(drone);
		drone.LOGGER_OK = ready;
	}
	return ready;
}

bool Logger::isReady() const {
	return ready;
}

bool Logger::isLogging() const {
	return logging;
}

bool Logger::buildLogPath(char* output, size_t outputSize) const {
	time_t now = time(nullptr);
	if (now > 1700000000) {
		struct tm timeInfo;
		localtime_r(&now, &timeInfo);
		char fileName[32];
		strftime(fileName, sizeof(fileName), "%Y%m%d_%H%M%S.csv", &timeInfo);
		snprintf(output, outputSize, "/logs/%s", fileName);
		return true;
	}

	snprintf(output, outputSize, "/logs/log_%010lu.csv", millis());
	return true;
}

bool Logger::openLogFile() {
	if (!ready) {
		return false;
	}

	buildLogPath(logPath, sizeof(logPath));
	logFile = SD.open(logPath, FILE_WRITE);
	if (!logFile) {
		return false;
	}

	writeHeader();
	logFile.println();
	logFile.flush();
	return true;
}

void Logger::writeSeparator() {
	logFile.print(',');
}

void Logger::writeBool(bool value) {
	logFile.print(value ? 1 : 0);
}

void Logger::writeInt32(int32_t value) {
	logFile.print(value);
}

void Logger::writeUInt8(uint8_t value) {
	logFile.print(value);
}

void Logger::writeFloat(float value, uint8_t digits) {
	logFile.print(value, digits);
}

void Logger::writeBasicHeader() {
	logFile.print("timestamp_s,dt_s,pitch_setpoint,pitch_measurement,pitch_control,roll_setpoint,roll_measurement,roll_control,yaw_setpoint,yaw_measurement,yaw_control");
}

void Logger::writeFullTelemetryHeader() {
	logFile.print(",flight_mode,attitude_pitch,attitude_yaw,attitude_roll,motor_m1,motor_m2,motor_m3,motor_m4,flight_pitch,flight_roll,flight_yaw,flight_throttle,altitude,GPS_OK,MOTOR_OK,PRESSURE_OK,IMU_OK,RADIO_OK,GROUND_LINK_OK,temp,pressure,batteryVoltage,latitude,longitude");
}

void Logger::writeHeader() {
	writeBasicHeader();
	if (includeFullTelemetry) {
		writeFullTelemetryHeader();
	}
}

void Logger::writeBasicSnapshot(const LogSnapshot& snapshot) {
	writeFloat(snapshot.timestampSeconds, 3);
	writeSeparator();
	writeFloat(snapshot.deltaSeconds, 3);
	writeSeparator();
	writeFloat(snapshot.flightControls.pitch, 3);
	writeSeparator();
	writeFloat(snapshot.attitude.pitch, 3);
	writeSeparator();
	writeFloat(snapshot.controlOutput.pitch, 3);
	writeSeparator();
	writeFloat(snapshot.flightControls.roll, 3);
	writeSeparator();
	writeFloat(snapshot.attitude.roll, 3);
	writeSeparator();
	writeFloat(snapshot.controlOutput.roll, 3);
	writeSeparator();
	writeFloat(snapshot.flightControls.yaw, 3);
	writeSeparator();
	writeFloat(snapshot.attitude.yaw, 3);
	writeSeparator();
	writeFloat(snapshot.controlOutput.yaw, 3);
}

void Logger::writeFullTelemetrySnapshot(const LogSnapshot& snapshot) {
	writeSeparator();
	writeUInt8(static_cast<uint8_t>(snapshot.mode));
	writeSeparator();
	writeFloat(snapshot.attitude.pitch, 3);
	writeSeparator();
	writeFloat(snapshot.attitude.yaw, 3);
	writeSeparator();
	writeFloat(snapshot.attitude.roll, 3);
	writeSeparator();
	writeInt32(snapshot.motorThrusts.m1);
	writeSeparator();
	writeInt32(snapshot.motorThrusts.m2);
	writeSeparator();
	writeInt32(snapshot.motorThrusts.m3);
	writeSeparator();
	writeInt32(snapshot.motorThrusts.m4);
	writeSeparator();
	writeFloat(snapshot.flightControls.pitch, 3);
	writeSeparator();
	writeFloat(snapshot.flightControls.roll, 3);
	writeSeparator();
	writeFloat(snapshot.flightControls.yaw, 3);
	writeSeparator();
	writeFloat(snapshot.flightControls.throttle, 3);
	writeSeparator();
	writeFloat(snapshot.altitude, 3);
	writeSeparator();
	writeBool(snapshot.gpsOk);
	writeSeparator();
	writeBool(snapshot.motorOk);
	writeSeparator();
	writeBool(snapshot.pressureOk);
	writeSeparator();
	writeBool(snapshot.imuOk);
	writeSeparator();
	writeBool(snapshot.radioOk);
	writeSeparator();
	writeBool(snapshot.groundLinkOk);
	writeSeparator();
	writeInt32(snapshot.telemetry.temp);
	writeSeparator();
	writeInt32(snapshot.telemetry.pressure);
	writeSeparator();
	writeInt32(snapshot.telemetry.batteryVoltage);
	writeSeparator();
	writeInt32(snapshot.telemetry.latitude);
	writeSeparator();
	writeInt32(snapshot.telemetry.longitude);
}

void Logger::writeSnapshot(const LogSnapshot& snapshot) {
	writeBasicSnapshot(snapshot);
	if (includeFullTelemetry) {
		writeFullTelemetrySnapshot(snapshot);
	}
	logFile.println();
}

Logger::LogSnapshot Logger::captureSnapshot() const {
	LogSnapshot snapshot{};
	const unsigned long now = millis();
	snapshot.timestampSeconds = static_cast<float>(now - logStartMs) / 1000.0f;
	snapshot.deltaSeconds = static_cast<float>(now - lastWriteMs) / 1000.0f;
	{
		DroneLockGuard droneLock(drone);
		snapshot.flightControls = drone.flightControls;
		snapshot.attitude = drone.attitude;
		snapshot.motorThrusts = drone.motorThrusts;
		snapshot.mode = drone.mode;
		snapshot.altitude = drone.altitude;
		snapshot.gpsOk = drone.GPS_OK;
		snapshot.motorOk = drone.MOTOR_OK;
		snapshot.pressureOk = drone.PRESSURE_OK;
		snapshot.imuOk = drone.IMU_OK;
		snapshot.radioOk = drone.RADIO_OK;
		snapshot.groundLinkOk = drone.GROUND_LINK_OK;
	}
	{
		TelemetryLockGuard telemetryLock(telemetry);
		snapshot.telemetry = static_cast<const TelemetryData&>(telemetry);
	}
	snapshot.controlOutput = motor.getControlOutput();
	return snapshot;
}

bool Logger::startLog(bool fullTelemetry) {
	if (!ready) {
		return false;
	}

	if (logging) {
		stopLog();
	}

	includeFullTelemetry = fullTelemetry;
	logStartMs = millis();
	lastWriteMs = logStartMs;
	if (!openLogFile()) {
		logging = false;
		return false;
	}

	logging = true;
	return true;
}

void Logger::stopLog() {
	if (!logging) {
		return;
	}

	logFile.flush();
	logFile.close();
	logging = false;
}

void Logger::loop() {
	if (!logging || !logFile) {
		return;
	}

	const LogSnapshot snapshot = captureSnapshot();
	writeSnapshot(snapshot);
	lastWriteMs = millis();
}