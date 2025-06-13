#include <RF24.h>
constexpr int CE_PIN {9};
constexpr int CSN_PIN {10};
const byte addressA[6] = "NodeA"; // 5-byte address + null
const byte addressB[6] = "NodeB";
const bool IS_NODE_A{true};  // Change to false for the other Arduino


class Transceiver {
	public:
	Transceiver();
	void loop();
	bool send(const char data[]);
	void setCallback(void (*callback)(const char* message));
	void receive();

	private:
	RF24 radio;
	char buffer[32];
	const int dataDelay {10};
	bool sendData {false};
	// std::function<void(const char*)> receiveCallback;
	void (*receiveCallback)(const char* message) = nullptr;
};