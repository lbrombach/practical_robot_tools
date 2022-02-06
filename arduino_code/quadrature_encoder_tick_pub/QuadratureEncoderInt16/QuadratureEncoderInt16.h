//from searching for "encoders" on this guys blog
//http://www.hessmer.org/blog/?s=quadrature&searchsubmit=
//
//The QuadratureEncoderInt16.h file needs to be copied into a folder called QuadratureEncoderInt16 under the
//libraries folder that is used by the Arduino environment so that it becomes accessible to Arduino programs.
/*
QuadratureEncoderInt.h - Quadrature encoder library
Designed to rollover/rollunder 16 bit int values (-32,768 to 32,767)
Written for ROS (ROSSERIAL) encoder_tick_pub.ino also by Lloyd Brombach
Lloyd Brombach, November 2020
lbrombach2@gmail.com

 Modified from QuadratureEncoder.h by
 Dr. Rainer Hessmer, April, 2010.
 Released into the public domain.
 
*/

#ifndef QuadratureEncoderInt16_h
#define QuadratureEncoderInt16_h

//#include "WProgram.h"
#include "Arduino.h"

class QuadratureEncoder
{
    /*
 Wraps the encoder setup and the update functions in a class
 
 !!! NOTE : User must call the functions OnAChanged and OnBChanged from an
 interrupt function him/herself!
 
 // ------------------------------------------------------------------------------------------------
 // Example usage :
 // ------------------------------------------------------------------------------------------------
 #include "QuadratureEncoder.h"
 
 QuadratureEncoder encoder(2, 3);
 
 void setup()
 {
 attachInterrupt(digitalPinToInterrupt(2), HandleInterruptA, CHANGE);
 attachInterrupt(digitalPinToInterrupt(3), HandleInterruptB, CHANGE);
 }
 
 void loop()
 {
 // do some stuff here - the joy of interrupts is that they take care of themselves
 }
 
 void HandleInterruptA()
 {
 encoder.OnAChanged();
 }
 
 void HandleInterruptB()
 {
 encoder.OnBChanged();
 }
 
 // ------------------------------------------------------------------------------------------------
 // Example usage end
 // ------------------------------------------------------------------------------------------------
 */

public:
    // pinA and pinB must be one of the external interupt pins
    QuadratureEncoder(int pinA, int pinB)
    {
        _PinA = pinA;
        _PinB = pinB;

        pinMode(_PinA, INPUT_PULLUP); // sets pin A as input
        pinMode(_PinB, INPUT_PULLUP); // sets pin B as input

        _ASet = digitalRead(_PinA); // read the input pin
        _BSet = digitalRead(_PinB); // read the input pin

        _Position = 0;
    }

    int getPosition() { return _Position; };
    void setPosition(int p) { _Position = p; };

    void countUp()
    {
        if (_Position == 32767)
        {
            _Position = -32768;
        }
        else
        {
            _Position++;
        }
    }

    void countDown()
    {
        if (_Position == -32768)
        {
            _Position = 32767;
        }
        else
        {
            _Position--;
        }
    }

    // Interrupt on A changing state
    void OnAChanged()
    {
        // Test transition
        _ASet = digitalRead(_PinA) == HIGH;
        // and adjust counter + if A leads B

        (_ASet != _BSet) ? countUp() : countDown();
    }

    // Interrupt on B changing state
    void OnBChanged()
    {
        // Test transition
        _BSet = digitalRead(_PinB) == HIGH;
        // and adjust counter + if B follows A
        (_ASet == _BSet) ? countUp() : countDown();
    }

    void SendInfo()
    {
        bool a = digitalRead(_PinA);
        bool b = digitalRead(_PinB);
        unsigned long milliSecs = millis();

        Serial.print(milliSecs);
        Serial.print("\t");
        Serial.print(a);
        Serial.print("\t");
        Serial.print(b);
        Serial.println();
    }

private:
    int _PinA, _PinB;
    bool _ASet, _BSet;
    int _Position;
};

#endif
