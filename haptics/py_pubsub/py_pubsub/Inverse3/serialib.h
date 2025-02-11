/*!
\file    serialib.h
\brief   Header file of the class serialib. This class is used for communication over a serial device.
\author  Philippe Lucidarme (University of Angers)
\version 2.0
\date    december the 27th of 2019
This Serial library is used to communicate through serial port.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

This is a licence-free software, it can be used by anyone who try to build a better world.

Adapted to work with basic_iostream, the stream functionality was adapted from https://github.com/Derecho/libserial/blob/master/src/SerialStreamBuf.cc
 
*/


#ifndef SERIALIB_H
#define SERIALIB_H

#if defined(__CYGWIN__)
    // This is Cygwin special case
    #include <sys/time.h>
#endif

// Include for windows
#if defined (_WIN32) || defined (_WIN64)
#if defined(__GNUC__)
    // This is MinGW special case
    #include <sys/time.h>
#else
    // sys/time.h does not exist on "actual" Windows
    #define NO_POSIX_TIME
#endif
    // Accessing to the serial port under Windows
    #include <windows.h>
#endif

#include <streambuf>

// Include for Linux
#if defined (__linux__) || defined(__APPLE__)
    #include <stdlib.h>
    #include <sys/types.h>
    #include <sys/shm.h>
    #include <termios.h>
    #include <string.h>
    #include <iostream>
    #include <sys/time.h>
    // File control definitions
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
#endif

/*! To avoid unused parameters */
#define UNUSED(x) (void)(x)

/**
 * number of serial data bits
 */
enum SerialDataBits {
    SERIAL_DATABITS_5, /**< 5 databits */
    SERIAL_DATABITS_6, /**< 6 databits */
    SERIAL_DATABITS_7, /**< 7 databits */
    SERIAL_DATABITS_8,  /**< 8 databits */
    SERIAL_DATABITS_16,  /**< 16 databits */
};

/**
 * number of serial stop bits
 */
enum SerialStopBits {
    SERIAL_STOPBITS_1, /**< 1 stop bit */
    SERIAL_STOPBITS_1_5, /**< 1.5 stop bits */
    SERIAL_STOPBITS_2, /**< 2 stop bits */
};

/**
 * type of serial parity bits
 */
enum SerialParity {
    SERIAL_PARITY_NONE, /**< no parity bit */
    SERIAL_PARITY_EVEN, /**< even parity bit */
    SERIAL_PARITY_ODD, /**< odd parity bit */
    SERIAL_PARITY_MARK, /**< mark parity */
    SERIAL_PARITY_SPACE /**< space bit */
};

/*!  \class     serialib
     \brief     This class is used for communication over a serial device.
*/
class serialib : public std::streambuf
{
public:

    //_____________________________________
    // ::: Constructors and destructors :::



    // Constructor of the class
    serialib    ();

    // Destructor
    ~serialib   ();



    //_________________________________________
    // ::: Configuration and initialization :::


    // Open a device
    char openDevice(const char *Device, const unsigned int Bauds,
                    SerialDataBits Databits = SERIAL_DATABITS_8,
                    SerialParity Parity = SERIAL_PARITY_NONE,
                    SerialStopBits Stopbits = SERIAL_STOPBITS_1);

    // Check device opening state
    bool isDeviceOpen() const;

    // Close the current device
    serialib* closeDevice();




    //___________________________________________
    // ::: Read/Write operation on characters :::


    // Write a char
    char writeChar(char);

    // Read a char (with timeout)
    char readChar(char *pByte,const unsigned int timeOut_ms=0);




    //________________________________________
    // ::: Read/Write operation on strings :::


    // Write a string
    char    writeString (const char *String);

    // Read a string (with timeout)
    int     readString  (   char *receivedString,
                            char finalChar,
                            unsigned int maxNbBytes,
                            const unsigned int timeOut_ms=0);



    // _____________________________________
    // ::: Read/Write operation on bytes :::


    // Write an array of bytes
    char    writeBytes  (const void *Buffer, const unsigned int NbBytes);

    // Read an array of byte (with timeout)
    int     readBytes   (void *buffer,unsigned int maxNbBytes,const unsigned int timeOut_ms=0, unsigned int sleepDuration_us=100);




    // _________________________
    // ::: Special operation :::


    // Empty the received buffer
    char    flushReceiver();

    virtual std::streambuf::int_type overflow(int_type c);
    virtual std::streambuf::int_type underflow();
    virtual std::streambuf::int_type uflow();
    virtual std::streamsize xsgetn(char_type *s, std::streamsize n);
    virtual std::streamsize xsputn(const char_type *s, std::streamsize n);
    virtual std::streamsize showmanyc();
    virtual std::streambuf::int_type pbackfail(int_type c);
    virtual std::streambuf* setbuf(char_type*, std::streamsize);

    // Return the number of bytes in the received buffer
    int     available();




    // _________________________
    // ::: Access to IO bits :::


    // Set CTR status (Data Terminal Ready, pin 4)
    bool    DTR(bool status);
    bool    setDTR();
    bool    clearDTR();

    // Set RTS status (Request To Send, pin 7)
    bool    RTS(bool status);
    bool    setRTS();
    bool    clearRTS();

    // Get RI status (Ring Indicator, pin 9)
    bool    isRI();

    // Get DCD status (Data Carrier Detect, pin 1)
    bool    isDCD();

    // Get CTS status (Clear To Send, pin 8)
    bool    isCTS();

    // Get DSR status (Data Set Ready, pin 9)
    bool    isDSR();

    // Get RTS status (Request To Send, pin 7)
    bool    isRTS();

    // Get CTR status (Data Terminal Ready, pin 4)
    bool    isDTR();


private:
    // Read a string (no timeout)
    int             readStringNoTimeOut  (char *String,char FinalChar,unsigned int MaxNbBytes);
    char mPutbackChar;
    // Current DTR and RTS state (can't be read on WIndows)
    bool            currentStateRTS;
    bool            currentStateDTR;
    bool mPutbackAvailable ;



#if defined (_WIN32) || defined( _WIN64)
    // Handle on serial device
    HANDLE          hSerial = INVALID_HANDLE_VALUE;
    // For setting serial port timeouts
    COMMTIMEOUTS    timeouts;
#endif
#if defined (__linux__) || defined(__APPLE__)
    int             fd;
#endif

};

inline serialib::serialib() : mPutbackChar(0),
                              mPutbackAvailable(false)
{
#if defined(_WIN32) || defined(_WIN64)
    // Set default value for RTS and DTR (Windows only)
    currentStateRTS = true;
    currentStateDTR = true;
    hSerial = INVALID_HANDLE_VALUE;
#endif
#if defined(__linux__) || defined(__APPLE__)
    fd = -1;
#endif
    setbuf(0, 0);
    return;
}

inline serialib::~serialib()
{
    if (this->isDeviceOpen())
    {
        this->closeDevice();
    }
    return;
}

inline bool serialib::isDeviceOpen() const
{
#if defined(_WIN32) || defined(_WIN64)
    return hSerial != INVALID_HANDLE_VALUE;
#endif
#if defined(__linux__) || defined(__APPLE__)
    return fd >= 0;
#endif
}

inline std::streambuf *serialib::setbuf(char_type *, std::streamsize)
{
    return std::streambuf::setbuf(0, 0);
}

inline serialib *serialib::closeDevice()
{
#if defined(_WIN32) || defined(_WIN64)
    CloseHandle(hSerial);
    hSerial = INVALID_HANDLE_VALUE;
#endif
#if defined(__linux__) || defined(__APPLE__)
    close(fd);
    fd = -1;
#endif
    return this;
}

inline std::streambuf::int_type serialib::uflow()
{
    int_type next_ch = underflow();
    mPutbackAvailable = false;
    return next_ch;
}

/*!  \class     timeOut
     \brief     This class can manage a timer which is used as a timeout.
   */
// Class timeOut
class timeOut
{
public:

    // Constructor
    timeOut();

    // Init the timer
    void                initTimer();

    // Return the elapsed time since initialization
    unsigned long int   elapsedTime_ms();

private:
#if defined (NO_POSIX_TIME)
    // Used to store the previous time (for computing timeout)
    LONGLONG       counterFrequency;
    LONGLONG       previousTime;
#else
    // Used to store the previous time (for computing timeout)
    struct timeval      previousTime;
#endif
};

#endif // serialib_H
