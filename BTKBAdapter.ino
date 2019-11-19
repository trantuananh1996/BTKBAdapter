// Reddit:    https://www.reddit.com/user/IBNai/
// Instagram: https://www.instagram.com/ib_nai/
// Facebook:  https://www.facebook.com/IB.Nye
// YouTube:   https://youtu.be/L7EqKJ02lK4

// https://www.elecrow.com/wiki/index.php?title=USB_Host_Shield_for_Arduino

#include <hidboot.h>
#include <usbhid.h>
#include <usbhub.h>
#include <hidcomposite.h>
#include <SoftwareSerial.h>

#include <avr/power.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

// Override HIDComposite to be able to select which interface we want to hook
// into
class BTHKBAdapter : public HIDComposite {
 public:
  BTHKBAdapter(USB* p) : HIDComposite(p){};

 protected:
  void Parse(USBHID* hid, uint8_t ep, bool is_rpt_id, uint8_t len,
             uint8_t* buf);  // Called by the HIDComposite library
  bool SelectInterface(uint8_t iface, uint8_t proto);
};

// Return true for the interface we want to hook into
bool BTHKBAdapter::SelectInterface(uint8_t iface, uint8_t proto) {
  if (proto != 0) return true;

  return false;
}

// Will be called for all HID data received from the USB interface
void BTHKBAdapter::Parse(USBHID* hid, uint8_t ep, bool is_rpt_id, uint8_t len,
                         uint8_t* buf) {
  for (uint8_t i = 0; i < len; i++) {
    Serial.write((uint8_t)buf[i]);
  }
  Serial.write("");
}

class KbdRptParser : public BTHKBAdapter {
 public:
  KbdRptParser(USB* p) : BTHKBAdapter(p){};
  void Parse(USBHID* hid, uint8_t ep, bool is_rpt_id, uint8_t Length,
             uint8_t* buf);
};

void KbdRptParser::Parse(USBHID* hid, uint8_t ep, bool is_rpt_id,
                         uint8_t Length, uint8_t* buf) {
  Serial.write((uint8_t)0xFD);
  Serial.write((uint8_t)Length + 1);
  Serial.write((uint8_t)0x01);
  BTHKBAdapter::Parse(hid, ep, is_rpt_id, Length, buf);
};

USB Usb;
// USBHub     Hub(&Usb);
uint32_t next_time;
KbdRptParser Prs(&Usb);

// For communicate with bluetooth module
SoftwareSerial bluetoothSerial(4, 5);  //(RX, TX)

void setup() {
  ADCSRA = 0;  // disable ADC by setting ADCSRA register to 0
  power_adc_disable();  // disable the clock to the ADC module
  Serial.begin(115200);
  bluetoothSerial.begin(115200);

#if !defined(__MIPSEL__)
  while (!Serial)
    ;  // Wait for serial port to connect - used on Leonardo, Teensy and other
       // boards with built-in USB CDC serial connection
#endif

  // Serial.println("Start");

  if (Usb.Init() == -1) Serial.println("OSC did not start.");
  // Set this to higher values to enable more debug information
  // minimum 0x00, maximum 0xff, default 0x80
  UsbDEBUGlvl = 0xff;
  delay(200);

  next_time = millis() + 5000;

  // HidKeyboard.SetReportParser(0, (HIDReportParser*)&Prs);
}

void loop() {
  Usb.Task();

  if (bluetoothSerial.available()) {
    Serial.print((char)bluetoothSerial.read());
  }
  if (Serial.available()) {
    bluetoothSerial.print((char)Serial.read());
  }
}
