void action() { 
/*********** Put what you want the code to do with the values (dmxvalue) here *************
example code: print out the received values to the serial port, and set PWM pins 5 and 6 
to the first two values received.  You can take this code out and put your own in.*/

//begin example code
for(byte j = 0; j < NUMBER_OF_CHANNELS; j++) {
  /* You'll need to use a program like Putty that allows you to set custom baud rates
   * in order to communicate with the Arduino over serial; the Arduino IDE only allows
   * you to pick common rates such as 9600.
   */
  Serial.print(dmxvalue[j]);
  }
  
  analogWrite(red_led, dmxvalue[1]);
  analogWrite(green_led, dmxvalue[2]);
  analogWrite(blue_led, dmxvalue[3]);
 
  return;  //go back to loop()
} //end action() loop
