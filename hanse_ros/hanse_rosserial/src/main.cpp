
#include <WProgram.h>
#include <HardwareSerial.h>

#include <ros.h>
#include "ros/node_handle.h"

#include "Logger.h"

ros::NodeHandle node;

void printBootReason() {
	if (bit_is_set(MCUSR,JTRF))
                Logger::info(PSTR("Reboot reason: JTAG"));
	if (bit_is_set(MCUSR,WDRF))
                Logger::info(PSTR("Reboot reason: Watchdog"));
	if (bit_is_set(MCUSR,BORF))
                Logger::info(PSTR("Reboot reason: Brown Out"));
	if (bit_is_set(MCUSR,EXTRF))
                Logger::info(PSTR("Reboot reason: External reset"));
	if (bit_is_set(MCUSR,PORF))
                Logger::info(PSTR("Reboot reason: Power On"));

}

void setup() {

        //node.getHardware()->setBaud(500000);
        node.initNode();

        Logger::info(PSTR("Node connected"));

        printBootReason();

        Logger::info(PSTR("Boot complete"));

	// clear boot reason register
        MCUSR = 0;
}

int main() {

	init();

	setup();

	uint32_t count=0;

        while (true) {

    	    // check for new messages from the other side
    	    node.spinOnce();

    	    if (++count % 100 == 0) {
                Logger::info(PSTR("Ran %ld iterations."), count);
    	    }

	}
	return 0;
}

