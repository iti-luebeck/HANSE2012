#ifndef CHECKSUM_H
#define CHECKSUM_H


// This computes a fletcher-16 checksum
class checksum {
    uint8_t a;
    uint8_t b;
public:
    checksum() : a(0), b(0) {};
    void reset() { a = 0; b = 0; }
    uint8_t add_byte(uint8_t byte)
    {
	a += byte;
	//	if (a < byte)
	//    a++;
	b += a;
	//if (b < a)
	//    b++;
	return byte;
    }
    uint8_t checksum_a() {return a;}
    uint8_t checksum_b() {return b;}
};

#endif
