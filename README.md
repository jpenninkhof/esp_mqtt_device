**ESP MQTT Switch**
==========
This firmware is meant to control a relay in e.g. a power outlet or power strip through MQTT.

The ESP8266 will register itself with the MQTT server and will listen to topic /DeviceX/\<chip-ID\>.  
Inbound message are expected to be formatted as JSON messages and will be parsed for switching instruction. 
Please find a valid JSON instruction below:

`{"switch":"off"}`

### Relay

The relay is supposed to be connected to ESP Pin GPIO2. To experiment with the firmware, a LED will of course also do.

### Push button

Optionally a push button can be connected meant to override messages from the MQTT broker, allowing you to physically switch the relay as well.
 
When the push button is pressed, the relay will change its state and a JSON message is sent to the MQTT server indicating its new state.

The optional push button should be connected to ESP Pin GPIO0 and when the button is pressed, this pin should be grounded.