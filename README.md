# P1Meter
my working P1 smart meter data reporting to MQTT/HASS
- copied heavily from here: https://github.com/jantenhove/P1-Meter-ESP8266
- more details and other resources at link above
# In short:
- use ESP8266 Huzzah from Adafruit: https://learn.adafruit.com/adafruit-huzzah-esp8266-breakout/overview
- I used pin15 on the Huzzah in this case for RxD. Ground is Ground and RTS to 3.3V as in the first link above.
- no pull-up or any other signal inverting trick is needed! This piece of code does everything! 
