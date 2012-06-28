#ifndef SLEEP_H
#define SLEEP_H

class sleep {
public:
    // assumes irqs are disabled when called
    static void wait_for_irq() {
	sleep_enable();
	NONATOMIC_BLOCK(NONATOMIC_FORCEOFF) {
	    sleep_cpu();
	    sleep_disable();
	}
    }
};

#endif
