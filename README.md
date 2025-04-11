ESP-IDF HumidityFrog
====================

HumidityFrog is an ESP-IDF ESP32-DHT11 sensor reporter over MQTT. 

I wanted something simple and cheap for my 3D printing lab and also track the effect of humidity on my allergies.

It uses a low-budget DHT11 sensor, not the most accurate thing in the world but able to get a sense of Dry/Normal/Damp and within a few degrees of temperature. It should mount inside the AMS filament spooler for my printer, or even have a few around the site. It's set up to publish on MQTT esp32/dht11 topic, but of course whatever it's up to you, modify as you do. I tested vs mosquitto broker, be sure your ufw/firewalls are permissive of TCP 1883. No TLS. 





Please check [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for getting started instructions.

*Code in this repository is in the Public Domain (or CC0 licensed, at your option.)
Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.*
