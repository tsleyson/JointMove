

/* ------------------------------------------------------------------------ --
--                                                                          --
--                        PC serial port connection object                  --
--                           for non-event-driven programs                  --
--                                                                          --
--                                                                          --
--                                                                          --
--  Copyright @ 2001          Thierry Schneider                             --
--                            thierry@tetraedre.com                         --
--                                                                          --
--                                                                          --
--                                                                          --
-- ------------------------------------------------------------------------ --
--                                                                          --
--  Filename : sertest2.cpp                                                 --
--  Author   : Thierry Schneider                                            --
--  Created  : April 4th 2000                                               --
--  Modified : April 8th 2001                                               --
--  Plateform: Windows 95, 98, NT, 2000 (Win32)                             --
-- ------------------------------------------------------------------------ --
--                                                                          --
--  This software is given without any warranty. It can be distributed      --
--  free of charge as long as this header remains, unchanged.               --
--                                                                          --
-- ------------------------------------------------------------------------ */




/* ---------------------------------------------------------------------- */
#ifndef TSERIAL_H
#define TSERIAL_H
#include <ostream>
#include <stdio.h>
#include <iostream>
#include <windows.h>
using namespace std;

enum serial_parity  { spNONE,    spODD, spEVEN };


/* -------------------------------------------------------------------- */
/* -----------------------------  Tserial  ---------------------------- */
/* -------------------------------------------------------------------- */
class Tserial
{

    // -------------------------------------------------------- //
protected:
	wchar_t           port[10];                      // port name "com1",...
    int               rate;                          // baudrate
    serial_parity     parityMode;
    HANDLE            serial_handle;                 // ...

    // ++++++++++++++++++++++++++++++++++++++++++++++
    // .................. EXTERNAL VIEW .............
    // ++++++++++++++++++++++++++++++++++++++++++++++
public:
    Tserial();
    ~Tserial();
    friend void operator << (Tserial& stream, char c);
    friend void operator << (Tserial& stream, char *ptr);
    friend void operator >> (Tserial& stream, char &c);
    friend void operator >> (Tserial& stream, char *ptr);
    int           connect          (wchar_t *port_arg, int rate_arg,
                                    serial_parity parity_arg);
    // sendChar and sendArray are used to send commands to the serial
    // port.
    void          sendChar         (char c);
    void          sendArray        (char *buffer, int len);
    // *buffer is a string that lists the command to the robot
    char          getChar          (void);
    int           getArray         (char *buffer, int len);
    int           getNbrOfBytes    (void);
    void          disconnect       (void);

    // friend char &operator<<();
};
/* -------------------------------------------------------------------- */

#endif TSERIAL_H


