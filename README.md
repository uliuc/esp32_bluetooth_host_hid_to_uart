This project is a small extension of Espressif's Bluetooth HID Host example. 
It does the following: it scans for available Bluetooth HID devices nearby and connects to all devices found. 
These must, of course, be in pairing mode. Afterwards, all events received from these devices are forwarded via UART. 

Why does this project exist? It was created during the porting of a Mac emulator (https://github.com/uliuc/minivmac_esp32) running on an ESP32S3. 
However, the ESP32S3 cannot handle classic Bluetooth, only BLE. Since I don't have any mice or keyboards with BLE, 
I had to use a simple ESP32 that forwards the data via UART. 

At the moment, the names of the devices to be connected must be adjusted in the source code. 

e.g. 
#if CONFIG_BT_HID_HOST_ENABLED
const int device_list_len = 2;
static const char * remote_device_names[] = { “Perixx_BT_Mouse”, “Bluetooth 5.0 Keyboard” };
#endif // CONFIG_BT_HID_HOST_ENABLED

or CONFIG_BT_HID_HOST_ENABLED must be deactivated.

The project was compiled with IDF version 5.5.
