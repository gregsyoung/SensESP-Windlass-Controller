<html><head><meta http-equiv="Content-Type" content="text/html; charset=utf-8" /><title></title></head><body><h1>SensESP Windlass Chain Counter</h1>
<p>****   NOTE - still under development ******</p>
<p>This Windlass Counter is a fork of …. <a href="https://github.com/raffmont/SensESP-Windlass-Controller">https://github.com/raffmont/SensESP-Windlass-Controller</a>
but with the “control” elements deleted , retaining just the “counter” and associated logic.
It requires electrical access to READ the Up &amp; Down control signals from the windlass, AND the chain counter sensor (reed or hall effect)
It utilises Expressif ESP32 WROM microcontroller and the SensESP framework (<a href="https://github.com/SignalK/SensESP">https://github.com/SignalK/SensESP</a>).</p>
<p>The device connects to an onboard Signal K server (<a href="https://signalk.org">https://signalk.org</a>), providing the following features:</p>
<ul>
<li>Count deployed anchor rode in meters.</li>
<li>Measure the up/down chain speed.</li>
<li>Sense the windlass status as going up, down, free down, free up, and off.</li>
<li>Enables remote/automatic windlass control reading the status.</li>
<li>Fully customizable Signal K keys and calibration parameters.</li>
<li>Local chain counter reset button</li>
<li>Remote reset of chain counter</li>
</ul>
<p>The device and all the software are provided with no warrinty or responsibility for correct or incorrect usage, eventually generating damages to people or freights.</p>
<p>This work has been inspired and forked from <a href="https://github.com/raffmont/SensESP-Windlass-Controller">https://github.com/raffmont/SensESP-Windlass-Controller</a>;
which was partially  inspired by this repository <a href="https://github.com/AK-Homberger/ESP8266_AnchorChainControl_WLAN">https://github.com/AK-Homberger/ESP8266_AnchorChainControl_WLAN</a>,  the SensESP example “Chain Counter”.</p>
</body></html>