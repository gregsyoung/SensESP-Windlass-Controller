SensESP Windlass Chain Counter

               ****   NOTE - still under development ******


This windlass counter/monitor provides output status and information of rode deployment to a SignalK server.  
It requires electrical access to READ the Up & Down control signals from the windlass switches/solenoids, AND the chain counter sensor (reed or hall effect) 
It utilises an Expressif ESP32  microcontroller and the SensESP framework (https://github.com/SignalK/SensESP).

The device connects to an onboard Signal K server (https://signalk.org), providing the following features:

* Anchor Rode deployed (meters)
* The up/down chain speed (m/s)
* Windlass status (direction) :  going up, down, and off.
* Fully customizable Signal K keys and calibration parameters. 
* Chain counter reset button (local) 
* Remote reset of chain counter (via SignalK path)


This software is provided "as is", without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose and noninfringement. in no event shall the authors or copyright holders be liable for any claim, damages or other liability, whether in an action of contract, tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in the software.


This Windlass Monitor was inspired and forked from a early version of https://github.com/raffmont/SensESP-Windlass-Controller; (with the "control" elements deleted , retaining just the "counter" and associated logic); which was partially inspired by this repository https://github.com/AK-Homberger/ESP8266_AnchorChainControl_WLAN,  the SensESP example "Chain Counter".