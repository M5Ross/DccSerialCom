#include <Arduino.h>


#ifndef HW_VERSION
#warning HW_VERSION not defined !
#define HW_VERSION	0
#endif
#ifndef SW_VERSION
#warning SW_VERSION not defined !
#define HW_VERSION	0
#endif
#ifndef DECODER_ID
#warning DECODER_ID not defined !
#define DECODER_ID	0
#endif


#if defined (__cplusplus)
	extern "C" {
#endif

extern void notifyExecuteFunction(uint8_t function, uint8_t state) __attribute__ ((weak));
extern uint16_t notifyGetCVnum(uint16_t index) __attribute__ ((weak));
extern uint16_t notifyGetCVval(uint16_t CV) __attribute__ ((weak));
extern void notifySetCV(uint16_t CV, uint16_t value) __attribute__ ((weak));
extern void notifySerialComEvent(uint8_t code, uint16_t value1, uint16_t value2) __attribute__ ((weak));

#if defined (__cplusplus)
}
#endif


class DccSerialCom {
private:
	#if defined(__AVR_ATmega32U4__)
	Serial_ * myPort;
	#else
	HardwareSerial * myPort;
	#endif
	
	uint16_t n_cv;
	uint16_t multi_adr;
	uint16_t single_inv;
	uint8_t __a;
	uint8_t __b;
	uint8_t __c;
	uint16_t __n;
	uint16_t __i;
	bool writeSerial = true;
	bool error = false;
	uint8_t reply[8];
	
	void split(uint16_t n);
	
public:
	DccSerialCom(uint16_t CVlistSize, uint16_t MultiAddressStart, uint16_t SingleInvertStart);
	DccSerialCom(uint16_t CVlistSize);
	#if defined(__AVR_ATmega32U4__)
	void init(Serial_ &comPort);
	#else
	void init(HardwareSerial &comPort);
	#endif
	void end(void);
	void process(void);
};

DccSerialCom::DccSerialCom(uint16_t CVlistSize) {
  n_cv = CVlistSize;
  multi_adr = 0;
  single_inv = 0;
}

DccSerialCom::DccSerialCom(uint16_t CVlistSize, uint16_t MultiAddressStart, uint16_t SingleInvertStart) {
  n_cv = CVlistSize;
  multi_adr = MultiAddressStart;
  single_inv = SingleInvertStart;
}

#if defined(__AVR_ATmega32U4__)
void DccSerialCom::init(Serial_ &comPort) {
  myPort = &comPort;
}
#else
void DccSerialCom::init(HardwareSerial &comPort) {
  myPort = &comPort;
}
#endif

void DccSerialCom::end(void) {
  #if defined(__AVR_ATmega32U4__)
  myPort = NULL;
  #else
  myPort = NULL;
  #endif
}

void DccSerialCom::split(uint16_t n) {
  __a = 0;
  __b = 0;
  __c = 0;
  if(n>99)
    __a = n/100;
  if(n>9)
    __b = (n-(__a*100))/10;
  __c = n-(__a*100)-(__b*10);
}

void DccSerialCom::process(void) {
  if(writeSerial) {
    writeSerial = false;
    error = false;
  }
  // aacccvvv
  // a: 00 = tipo decoder
  //    01 = hw ver
  //    02 = sw ver
  //    10 = scrivi cv
  //    11 = leggi cv
  //    20 = sim attivo dx
  //    21 = sim attivo sx
  //    30 = numero cv attive
  //    31 = leggi cv in serie
  //    32 = multi adr start
  //    33 = single inv start
  //    98 = ok
  //    99 = errore
  // c: 000-999 = n° cv
  // v: 000-999 = val cv
  if(myPort->available()>7) {
    uint8_t val[8];
    for(uint8_t i=0; i<8; i++) {
      val[i] = myPort->read()-'0';
      reply[i] = 0;
    }
    //myPort->flush();
    uint8_t aa = val[0]*10 + val[1];
    uint16_t ccc = val[2]*100 + val[3]*10 + val[4];
    uint16_t vvv = val[5]*100 + val[6]*10 + val[7];
    if(ccc > 999 || vvv > 999)
      error = true;

    reply[0] = 9;
    reply[1] = 8;
    if(error)
      reply[1] = 9;
  
	if (notifySerialComEvent)
		notifySerialComEvent(aa, ccc, vvv);

    switch(aa) {
      case 0:
        split(DECODER_ID);
        reply[5] = __a;
        reply[6] = __b;
        reply[7] = __c;
        break;
      case 1:
        split(HW_VERSION);
        reply[5] = __a;
        reply[6] = __b;
        reply[7] = __c;
		break;
      case 2:
        split(SW_VERSION);
        reply[5] = __a;
        reply[6] = __b;
        reply[7] = __c;
        break;
      case 10:
        if (notifySetCV)
			notifySetCV(ccc, vvv);
        break;
      case 11:
	    vvv = 0;
	    if (notifyGetCVval)
			vvv = notifyGetCVval(ccc);
		if (vvv > 999)
			vvv = 999;
        split(vvv);
        reply[5] = __a;
        reply[6] = __b;
        reply[7] = __c;
        break;
      case 20:
	  case 21:
		if (notifyExecuteFunction)
			notifyExecuteFunction(ccc, aa-20);
        break;
      case 30:
        __n = n_cv;
        __i = 0;
        split(__n);
        reply[5] = __a;
        reply[6] = __b;
        reply[7] = __c;
        break;
      case 31:
		vvv = __i;
		if (notifyGetCVnum)
			vvv = notifyGetCVnum(__i);
		if (vvv > 999)
			vvv = 999;
        split(vvv);
        reply[2] = __a;
        reply[3] = __b;
        reply[4] = __c;
		if (notifyGetCVval)
			vvv = notifyGetCVval(vvv);
		if (vvv > 999)
			vvv = 999;
        split(vvv);
        reply[5] = __a;
        reply[6] = __b;
        reply[7] = __c;
        __i++;
        break;
	  case 32:
        __n = multi_adr;
        __i = 0;
		split(__n);
        reply[5] = __a;
        reply[6] = __b;
        reply[7] = __c;
        break;
	  case 33:
        __n = single_inv;
        __i = 0;
		split(__n);
        reply[5] = __a;
        reply[6] = __b;
        reply[7] = __c;
        break;
      default:
        reply[1] = 9;
        break;
    }
	
	for(uint8_t i=0; i<8; i++)
      myPort->print(reply[i]);
	  
	writeSerial = true;
  }
}
