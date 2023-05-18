#ifndef ALPHA_DRIVER_DEFAULT_H
#define ALPHA_DRIVER_DEFAULT_H

/*****************************************************************************/
/***                                 Serial                                ***/
/*****************************************************************************/
#define DEFAULT_PORT "/dev/ttyACM0"      // Default port name
#define DEFAULT_BAUD 115200              // Default baudate
#define DEFAULT_PARITY 0                 // Default parity: parity_none
#define DEFAULT_DATABITS 8               // Default databits
#define DEFAULT_STOPBITS 1               // Default stopbits
#define DEFAULT_TIMEOUT 1000             // Default timeout

/*****************************************************************************/
/***                               Thruster                                ***/
/*****************************************************************************/
#define DEFAULT_CHANNELS 1
#define DEFAULT_SAFETY_TIMEOUT 3
#define DEFAULT_SAFETY_RATE 10

#endif // ALPHA_DRIVER_DEFAULT_H