# LoRa_Mqtt_Transceiver

Project is based on LoRa Receiver &amp; Sender code for Kincony ALR Platform (https://shop.kincony.com/products/esp32-lora-sx1278-gateway-kincony-alr)
![image](https://github.com/user-attachments/assets/ffaa3cec-87c2-4d95-83b1-080c13ab0c69)
![image](https://github.com/user-attachments/assets/e637d592-fe64-455e-8915-ed1a993c0131)

The role is depending on the DIP Switch position

If all are left (00000000) or right (11111111) then role is "receiver"
 The receiver is receiving lora messages from the sender nodes and forwards the mqtt topics to the mqtt broker in homeassistant. If a new adress is located, then the mqtt topics are send to register the new device in the broker
    
All other dipswitch positions are setting the role "sender".
 As sender, all signals from the hardware (4 analog, 2 digital ) are sent by LoRa to the receivers, every 10 seconds. Sender also awaits Lora commands, to toggle the relay.

Details to hardware: https://www.kincony.com/forum/forumdisplay.php?fid=59
