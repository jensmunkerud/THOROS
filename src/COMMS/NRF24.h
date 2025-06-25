#include <RF24.h>
constexpr int CE_PIN {9};
constexpr int CSN_PIN {10};
const byte addressA[5] = {'N', 'A', '0', '0', '1'};
const byte addressB[5] = {'N', 'B', '0', '0', '2'};
constexpr bool IS_NODE_A{true};  // Change to false for the other Arduino


class NRF24 {
	public:
	NRF24();
	void begin();
	void loop();
	bool send(const char data[]);
	void setCallback(void (*callback)(const char* message));
	void receive();
	RF24 radio;

	private:
	void (*receiveCallback)(const char* message) = nullptr;
};