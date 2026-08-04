#include <Arduino.h>

#if defined(CONFIG_IDF_TARGET_ESP32)

#include <BluetoothSerial.h>
#include <micro_ros_arduino.h>

extern "C"
{

  static BluetoothSerial esp32_bluetooth_spp_client;

  bool arduino_esp32_bluetooth_spp_transport_open(struct uxrCustomTransport * transport)
  {
    struct micro_ros_esp32_bluetooth_spp_params * params =
      (struct micro_ros_esp32_bluetooth_spp_params *) transport->args;

    return esp32_bluetooth_spp_client.begin(params->device_name);
  }

  bool arduino_esp32_bluetooth_spp_transport_close(struct uxrCustomTransport * transport)
  {
    (void) transport;
    esp32_bluetooth_spp_client.end();
    return true;
  }

  size_t arduino_esp32_bluetooth_spp_transport_write(
    struct uxrCustomTransport * transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *errcode)
  {
    (void) transport;
    (void) errcode;

    size_t sent = esp32_bluetooth_spp_client.write(buf, len);
    esp32_bluetooth_spp_client.flush();

    return sent;
  }

  size_t arduino_esp32_bluetooth_spp_transport_read(
    struct uxrCustomTransport * transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *errcode)
  {
    (void) transport;
    (void) errcode;

    esp32_bluetooth_spp_client.setTimeout(timeout);
    return esp32_bluetooth_spp_client.readBytes((char *)buf, len);
  }
}

#endif
