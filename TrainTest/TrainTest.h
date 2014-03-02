/*
* TrainTest.h
*
* Created: 01/03/2014 16:21:12
*  Author: Luke
*/


#ifndef TRAINTEST_H_
#define TRAINTEST_H_

//

#define Orb(port,bitnum)		port |= _BV(bitnum)
#define Setb(port,bitnum)		port |= _BV(bitnum)
#define Clrb(port,bitnum)		port &= ~(_BV(bitnum))
#define Rdb(pinp,bitnum)		(pinp & _BV(bitnum))
#define Invert(port,bitnum)		(port) ^= _BV(bitnum)


#endif /* TRAINTEST_H_ */