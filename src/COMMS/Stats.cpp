class Stats {
	public:
	int baroTemp {0};
	int baroHeight {0};
	int gyroTemp {0};
	bool systemsOk {false};		// whether ALL systems is up and running
	bool criticalOk {false};	// whether CRITICAL systems is up and running

	Stats();
};
