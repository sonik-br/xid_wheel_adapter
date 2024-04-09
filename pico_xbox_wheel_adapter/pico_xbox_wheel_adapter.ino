/*******************************************************************************
 * Racing Wheel to Xbox (XID) Adapter
 * By Matheus Fraguas (sonik-br)
 * https://github.com/sonik-br/xid_wheel_adapter
 * 
 * It will also double as a genetic XINPUT adapter to XID
*******************************************************************************/

#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "xinput_host.h"
#include "xid.h"
#include "xid_driver.h"
#include "wheel_ids.h"
#include "wheel_reports.h"

// USB D+ pin
// D- must be DP +1
#define PIN_USB_HOST_DP 0


#ifndef CFG_TUH_XINPUT
  #error CFG_TUH_XINPUT not enabled. define it at tusb_config.h
#endif

const uint8_t wheel_8bits = 0;
const uint8_t wheel_10bits = 1;
const uint8_t wheel_14bits = 2;
const uint8_t wheel_16bits = 3;

// report to hold input from any wheel
generic_report_t generic_report;

uint8_t mounted_dev = 0;
uint8_t mounted_instance = 0;
bool mounted_is_xinput = false;

USB_XboxWheel_InReport_t xpad_data;

uint8_t xinput_out_rhport = 0;
bool tud_xinput_ready();

class Adafruit_USBD_XInput : public Adafruit_USBD_Interface {
  public:
    Adafruit_USBD_XInput();
    bool begin(void);
    virtual uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf, uint16_t bufsize);
};

Adafruit_USBD_XInput::Adafruit_USBD_XInput() { }

bool Adafruit_USBD_XInput::begin(void) {
  if (!TinyUSBDevice.addInterface(*this)) {
    return false;
  }

  //TinyUSBDevice.setID(0x045E, 0x0289);
  TinyUSBDevice.setDeviceVersion(0x0100);
  TinyUSBDevice.setVersion(0x0110);

  return true;
}


uint16_t Adafruit_USBD_XInput::getInterfaceDescriptor(uint8_t itfnum_deprecated, uint8_t *buf, uint16_t bufsize) {
  uint8_t itfnum = 0;
  uint8_t ep_in = 0;
  uint8_t ep_out = 0;

  // null buffer is used to get the length of descriptor only
  if (buf) {
    itfnum = TinyUSBDevice.allocInterface(1);
    ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
    ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
  }

  const uint8_t desc[] = { TUD_XID_WHEEL_DESCRIPTOR(itfnum, ep_out, ep_in) };
  const uint16_t len = sizeof(desc);

  if (bufsize < len)
    return 0;

  memcpy(buf, desc, len);
  return len;
}


Adafruit_USBH_Host USBHost;

Adafruit_USBD_XInput *_xinput;

void set_led(bool value) {
  #ifdef LED_BUILTIN
    gpio_put(LED_BUILTIN, value);
  #endif
}

void setup() {
  rp2040.enableDoubleResetBootloader();
  Serial.end();
  
  //Configure led pin
  #ifdef LED_BUILTIN
    gpio_init(LED_BUILTIN);
    gpio_set_dir(LED_BUILTIN, 1);
    gpio_put(LED_BUILTIN, LOW);
  #endif


  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
    while ( !Serial ) delay(10);   // wait for native usb
    Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
    while (1) delay(1);
  }

  _xinput = new Adafruit_USBD_XInput();
  
  _xinput->begin();
//  while( !TinyUSBDevice.mounted() ) delay(1);

    memset(&xpad_data, 0x00, sizeof(xpad_data));
    xpad_data.bLength = sizeof(xpad_data);


  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  /* https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/46 */
  pio_cfg.sm_tx      = 3;
  pio_cfg.sm_rx      = 2;
  pio_cfg.sm_eop     = 3;
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
  pio_cfg.tx_ch      = 9;
#endif /* ARDUINO_RASPBERRY_PI_PICO_W */

  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  USBHost.begin(1);
}

void loop() {
  USBHost.task();
  
  uint8_t index = xid_get_index_by_type(0, XID_TYPE_WHEEL);
  static USB_XboxWheel_InReport_t last_xpad_data {0x00};

  if (memcmp(&last_xpad_data, &xpad_data, sizeof(xpad_data))) {
    memcpy(&last_xpad_data, &xpad_data, sizeof(xpad_data));
    if (xid_send_report_ready(index))
    {
      if (!xid_send_report(index, &xpad_data, sizeof(xpad_data))) {
        TU_LOG1("[USBD] Error sending OUT report\r\n");
      }
    }
  }

  USB_XboxWheel_OutReport_t xpad_rumble;
  static uint16_t old_rumble_l, old_rumble_r;
  if (xid_get_report(index, &xpad_rumble, sizeof(xpad_rumble))) {
    bool update_needed = false;
    if (xpad_rumble.lValue != old_rumble_l || xpad_rumble.rValue != old_rumble_r) {
      old_rumble_l = xpad_rumble.lValue;
      old_rumble_r = xpad_rumble.rValue;
      update_needed = true;
    }
    if (update_needed) {
      uint8_t l_rumble, r_rumble;
      l_rumble = xpad_rumble.lValue >> 8;
      r_rumble = xpad_rumble.rValue >> 8;

      //XBONE have rumble values of 0-100;
      if (mounted_is_xinput) {
        l_rumble = (uint32_t)l_rumble * 100 / 0xFF;
        r_rumble = (uint32_t)r_rumble * 100 / 0xFF;
        
        if (mounted_dev != 0)
          tuh_xinput_set_rumble(mounted_dev, mounted_instance, l_rumble, r_rumble, true);
      }
    }
  }
}


//usbh_class_driver_t const* usbh_app_driver_get_cb(uint8_t* driver_count){
const usbh_class_driver_t *usbh_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = 1;
  return &usbh_xinput_driver;
}


void map_input(uint8_t const* report) {
  uint16_t vid, pid;
  tuh_vid_pid_get(mounted_dev, &vid, &pid);

  if (pid == pid_df) { // Driving Force. most logitech wheels will start in this mode

    // map the received report to the generic report
    df_report_t* input_report = (df_report_t*)report;

    generic_report.wheel_precision = wheel_10bits;
    generic_report.pedals_precision_16bits = false;

    generic_report.wheel_10 = input_report->wheel;
    generic_report.gasPedal_8 = input_report->gasPedal;
    generic_report.brakePedal_8 = input_report->brakePedal;

    generic_report.hat = input_report->hat;
    generic_report.cross = input_report->cross;
    generic_report.square = input_report->square;
    generic_report.circle = input_report->circle;
    generic_report.triangle = input_report->triangle;
    generic_report.R1 = input_report->R1;
    generic_report.L1 = input_report->L1;      
    generic_report.R2 = input_report->R2;
    generic_report.L2 = input_report->L2;
    generic_report.R3 = input_report->R3;
    generic_report.L3 = input_report->L3;
    generic_report.select = input_report->select;
    generic_report.start = input_report->start;

  } else if (pid == pid_dfp) { // Driving Force Pro UNTESTED
    
    // map the received report to the generic report
    dfp_report_t* input_report = (dfp_report_t*)report;
    
    generic_report.wheel_precision = wheel_14bits;
    generic_report.pedals_precision_16bits = false;
    
    generic_report.wheel_14 = input_report->wheel;
    generic_report.gasPedal_8 = input_report->gasPedal;
    generic_report.brakePedal_8 = input_report->brakePedal;

    generic_report.hat = input_report->hat;
    generic_report.cross = input_report->cross;
    generic_report.square = input_report->square;
    generic_report.circle = input_report->circle;
    generic_report.triangle = input_report->triangle;
    generic_report.R1 = input_report->R1;
    generic_report.L1 = input_report->L1;
    generic_report.R2 = input_report->R2;
    generic_report.L2 = input_report->L2;
    generic_report.R3 = input_report->R3;
    generic_report.L3 = input_report->L3;
    generic_report.select = input_report->select;
    generic_report.start = input_report->start;
    generic_report.gear_minus = input_report->gear_minus;
    generic_report.gear_plus = input_report->gear_plus;

  } else if (pid == pid_dfgt) { // Driving Force GT

    // map the received report to the generic report
    dfgt_report_t* input_report = (dfgt_report_t*)report;

    generic_report.wheel_precision = wheel_14bits;
    generic_report.pedals_precision_16bits = false;
    
    generic_report.wheel_14 = input_report->wheel;
    generic_report.gasPedal_8 = input_report->gasPedal;
    generic_report.brakePedal_8 = input_report->brakePedal;

    generic_report.hat = input_report->hat;
    generic_report.cross = input_report->cross;
    generic_report.square = input_report->square;
    generic_report.circle = input_report->circle;
    generic_report.triangle = input_report->triangle;
    generic_report.R1 = input_report->R1;
    generic_report.L1 = input_report->L1;
    generic_report.R2 = input_report->R2;
    generic_report.L2 = input_report->L2;
    generic_report.R3 = input_report->R3;
    generic_report.L3 = input_report->L3;
    generic_report.select = input_report->select;
    generic_report.start = input_report->start;
    generic_report.gear_minus = input_report->gear_minus;
    generic_report.gear_plus = input_report->gear_plus;
    generic_report.dial_cw = input_report->dial_cw;
    generic_report.dial_ccw = input_report->dial_ccw;
    generic_report.enter = input_report->enter;
    generic_report.plus = input_report->plus;
    generic_report.minus = input_report->minus;
    generic_report.horn = input_report->horn;
    generic_report.PS = input_report->PS;

  } else if (pid == pid_g25) { // G25

    // map the received report to output report
    g25_report_t* input_report = (g25_report_t*)report;

    generic_report.wheel_precision = wheel_14bits;
    generic_report.pedals_precision_16bits = false;
    
    generic_report.wheel_14 = input_report->wheel;
    generic_report.gasPedal_8 = input_report->gasPedal;
    generic_report.brakePedal_8 = input_report->brakePedal;
    generic_report.clutchPedal_8 = input_report->clutchPedal;

    generic_report.hat = input_report->hat;
    generic_report.cross = input_report->cross;
    generic_report.square = input_report->square;
    generic_report.circle = input_report->circle;
    generic_report.triangle = input_report->triangle;
    generic_report.R1 = input_report->R1;
    generic_report.L1 = input_report->L1;
    generic_report.R2 = input_report->R2;
    generic_report.L2 = input_report->L2;
    generic_report.R3 = input_report->R3;
    generic_report.L3 = input_report->L3;
    generic_report.select = input_report->select;
    generic_report.start = input_report->start;
    generic_report.shifter_x = input_report->shifter_x;
    generic_report.shifter_y = input_report->shifter_y;
    generic_report.shifter_1 = input_report->shifter_1;
    generic_report.shifter_2 = input_report->shifter_2;
    generic_report.shifter_3 = input_report->shifter_3;
    generic_report.shifter_4 = input_report->shifter_4;
    generic_report.shifter_5 = input_report->shifter_5;
    generic_report.shifter_6 = input_report->shifter_6;
    generic_report.shifter_r = input_report->shifter_r;
    generic_report.shifter_stick_down = input_report->shifter_stick_down;
    
  } else if (pid == pid_g27) { // G27

    // map the received report to output report
    g27_report_t* input_report = (g27_report_t*)report;

    generic_report.wheel_precision = wheel_14bits;
    generic_report.pedals_precision_16bits = false;
    
    generic_report.wheel_14 = input_report->wheel;
    generic_report.gasPedal_8 = input_report->gasPedal;
    generic_report.brakePedal_8 = input_report->brakePedal;
    generic_report.clutchPedal_8 = input_report->clutchPedal;

    generic_report.hat = input_report->hat;
    generic_report.cross = input_report->cross;
    generic_report.square = input_report->square;
    generic_report.circle = input_report->circle;
    generic_report.triangle = input_report->triangle;
    generic_report.R1 = input_report->R1;
    generic_report.L1 = input_report->L1;
    generic_report.R2 = input_report->R2;
    generic_report.L2 = input_report->L2;
    generic_report.R3 = input_report->R3;
    generic_report.L3 = input_report->L3;
    generic_report.R4 = input_report->R4;
    generic_report.L4 = input_report->L4;
    generic_report.R5 = input_report->R5;
    generic_report.L5 = input_report->L5;
    generic_report.select = input_report->select;
    generic_report.start = input_report->start;
    generic_report.shifter_x = input_report->shifter_x;
    generic_report.shifter_y = input_report->shifter_y;
    generic_report.shifter_1 = input_report->shifter_1;
    generic_report.shifter_2 = input_report->shifter_2;
    generic_report.shifter_3 = input_report->shifter_3;
    generic_report.shifter_4 = input_report->shifter_4;
    generic_report.shifter_5 = input_report->shifter_5;
    generic_report.shifter_6 = input_report->shifter_6;
    generic_report.shifter_r = input_report->shifter_r;
    generic_report.shifter_stick_down = input_report->shifter_stick_down;

  } else if (pid == pid_g29) { // G29

    // map the received report to output report
    g29_report_t* input_report = (g29_report_t*)report;

    generic_report.wheel_precision = wheel_16bits;
    generic_report.pedals_precision_16bits = false;

    generic_report.wheel_16 = input_report->wheel;
    generic_report.gasPedal_8 = input_report->gasPedal;
    generic_report.brakePedal_8 = input_report->brakePedal;
    generic_report.clutchPedal_8 = input_report->clutchPedal;

    generic_report.hat = input_report->hat;
    generic_report.cross = input_report->cross;
    generic_report.square = input_report->square;
    generic_report.circle = input_report->circle;
    generic_report.triangle = input_report->triangle;
    generic_report.R1 = input_report->R1;
    generic_report.L1 = input_report->L1;
    generic_report.R2 = input_report->R2;
    generic_report.L2 = input_report->L2;
    generic_report.select = input_report->share;
    generic_report.start = input_report->options;
    generic_report.R3 = input_report->R3;
    generic_report.L3 = input_report->L3;
    generic_report.shifter_1 = input_report->shifter_1;
    generic_report.shifter_2 = input_report->shifter_2;
    generic_report.shifter_3 = input_report->shifter_3;
    generic_report.shifter_4 = input_report->shifter_4;
    generic_report.shifter_5 = input_report->shifter_5;
    generic_report.shifter_6 = input_report->shifter_6;
    generic_report.shifter_r = input_report->shifter_r;
    generic_report.plus = input_report->plus;
    generic_report.minus = input_report->minus;
    generic_report.dial_cw = input_report->dial_cw;
    generic_report.dial_ccw = input_report->dial_ccw;
    generic_report.enter = input_report->enter;
    generic_report.PS = input_report->PS;
    generic_report.shifter_x = input_report->shifter_x;
    generic_report.shifter_y = input_report->shifter_y;
    generic_report.shifter_stick_down = input_report->shifter_stick_down;

  } else if (pid == pid_ffgp) { // Formula Force GP

    // map the received report to output report
    ffgp_report_t* input_report = (ffgp_report_t*)report;

    generic_report.wheel_precision = wheel_10bits;
    generic_report.pedals_precision_16bits = false;
    
    generic_report.wheel_10 = input_report->wheel;
    generic_report.gasPedal_8 = input_report->gasPedal;
    generic_report.brakePedal_8 = input_report->brakePedal;
    
    generic_report.cross = input_report->cross;
    generic_report.square = input_report->square;
    generic_report.circle = input_report->circle;
    generic_report.triangle = input_report->triangle;
    generic_report.R1 = input_report->R1;
    generic_report.L1 = input_report->L1;
  }
}

void map_output() {
  // shift axis values

  int16_t wheel = 0;
  uint8_t gas;
  uint8_t brake;

  if (generic_report.wheel_precision == wheel_8bits) {
    wheel = map(generic_report.wheel_8, 0, UINT8_MAX, INT16_MIN, INT16_MAX);
  } else if (generic_report.wheel_precision == wheel_10bits) {
    wheel = map(generic_report.wheel_10, 0, 1023UL, INT16_MIN, INT16_MAX);
  } else if (generic_report.wheel_precision == wheel_14bits) {
    wheel = map(generic_report.wheel_14, 0, 16383UL, INT16_MIN, INT16_MAX);
  } else { // wheel_16bits
    wheel = map(generic_report.wheel_16, 0, UINT16_MAX, INT16_MIN, INT16_MAX);
  }

  if (generic_report.pedals_precision_16bits) {
    gas = generic_report.gasPedal_16 >> 8;
    brake = generic_report.brakePedal_16 >> 8;
  } else {
    gas = generic_report.gasPedal_8;
    brake = generic_report.brakePedal_8;
  }

  xpad_data.dButtons = 0;

  switch (generic_report.hat) {
    case 0x0:
      xpad_data.dButtons |= XID_DUP;
      break;
    case 0x1:
      xpad_data.dButtons |= XID_DUP;
      xpad_data.dButtons |= XID_DRIGHT;
      break;
    case 0x2:
      xpad_data.dButtons |= XID_DRIGHT;
      break;
    case 0x3:
      xpad_data.dButtons |= XID_DDOWN;
      xpad_data.dButtons |= XID_DRIGHT;
      break;
    case 0x4:
      xpad_data.dButtons |= XID_DDOWN;
      break;
    case 0x5:
      xpad_data.dButtons |= XID_DDOWN;
      xpad_data.dButtons |= XID_DLEFT;
      break;
    case 0x6:
      xpad_data.dButtons |= XID_DLEFT;
      break;
    case 0x7:
      xpad_data.dButtons |= XID_DUP;
      xpad_data.dButtons |= XID_DLEFT;
      break;
    default:
      break;
  }

  if (generic_report.start)   xpad_data.dButtons |= XID_START;
  if (generic_report.select)  xpad_data.dButtons |= XID_BACK;
  if (generic_report.L3)      xpad_data.dButtons |= XID_LS;
  if (generic_report.R3)      xpad_data.dButtons |= XID_RS;

  xpad_data.A = (generic_report.cross || generic_report.L1)   ? 0xFF : 0; // gear down
  xpad_data.B = (generic_report.circle)                       ? 0xFF : 0;
  xpad_data.X = (generic_report.square || generic_report.R1)  ? 0xFF : 0; // gear up
  xpad_data.Y = (generic_report.triangle)                     ? 0xFF : 0;

  xpad_data.BLACK = (generic_report.L2) ? 0xFF : 0;
  xpad_data.WHITE = (generic_report.R2) ? 0xFF : 0;
  xpad_data.L     = ~brake;
  xpad_data.R     = ~gas;

  xpad_data.leftStickX = wheel;
  xpad_data.leftStickY = 0;

  xpad_data.rightStickX = 0;
  xpad_data.rightStickY = 0;
}





void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report_desc, uint16_t desc_len) {
  uint16_t vid;
  uint16_t pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  if ((vid == 0x046d) && ((pid & 0xff00) == 0xc200)) { // device is a logitech wheel
    set_led(HIGH);
    
    mounted_dev = dev_addr;
    mounted_instance = idx;
    mounted_is_xinput = false;

    memset(&generic_report, 0, sizeof(generic_report));
    tuh_hid_receive_report(dev_addr, idx);
  }
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t idx) {
  set_led(LOW);
  mounted_dev = 0;
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len) {

  // safe check
  if (len > 0 && dev_addr == mounted_dev && idx == mounted_instance) {
    // map the received report to generic_output
    map_input(report);

    // now map the generic_output to the output_mode
    map_output();
  }

  // receive next report
  tuh_hid_receive_report(dev_addr, idx);
}


void tuh_xinput_report_received_cb(uint8_t dev_addr, uint8_t instance, xinputh_interface_t const* xid_itf, uint16_t len) {
  const xinput_gamepad_t *p = &xid_itf->pad;

  if (xid_itf->last_xfer_result == XFER_RESULT_SUCCESS) {

    if (xid_itf->connected && xid_itf->new_pad_data) {
      xpad_data.dButtons = 0;
      if (p->wButtons & XINPUT_GAMEPAD_DPAD_UP)     xpad_data.dButtons |= XID_DUP;
      if (p->wButtons & XINPUT_GAMEPAD_DPAD_DOWN)   xpad_data.dButtons |= XID_DDOWN;
      if (p->wButtons & XINPUT_GAMEPAD_DPAD_LEFT)   xpad_data.dButtons |= XID_DLEFT;
      if (p->wButtons & XINPUT_GAMEPAD_DPAD_RIGHT)  xpad_data.dButtons |= XID_DRIGHT;
      if (p->wButtons & XINPUT_GAMEPAD_START)       xpad_data.dButtons |= XID_START;
      if (p->wButtons & XINPUT_GAMEPAD_BACK)        xpad_data.dButtons |= XID_BACK;
      if (p->wButtons & XINPUT_GAMEPAD_LEFT_THUMB)  xpad_data.dButtons |= XID_LS;
      if (p->wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) xpad_data.dButtons |= XID_RS;

      xpad_data.A = (p->wButtons & XINPUT_GAMEPAD_A) ? 0xFF : 0;
      xpad_data.B = (p->wButtons & XINPUT_GAMEPAD_B) ? 0xFF : 0;
      xpad_data.X = (p->wButtons & XINPUT_GAMEPAD_X) ? 0xFF : 0;
      xpad_data.Y = (p->wButtons & XINPUT_GAMEPAD_Y) ? 0xFF : 0;

      xpad_data.BLACK = (p->wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) ? 0xFF : 0;
      xpad_data.WHITE = (p->wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) ? 0xFF : 0;
      xpad_data.L     = p->bLeftTrigger;
      xpad_data.R     = p->bRightTrigger;

      xpad_data.leftStickX = p->sThumbLX;
      xpad_data.leftStickY = p->sThumbLY;

      xpad_data.rightStickX = p->sThumbRX;
      xpad_data.rightStickY = p->sThumbRY;
    }
  }
  
  tuh_xinput_receive_report(dev_addr, instance);
}

void tuh_xinput_mount_cb(uint8_t dev_addr, uint8_t instance, const xinputh_interface_t *xinput_itf) {
  set_led(HIGH);
  uint16_t PID, VID;
  tuh_vid_pid_get(dev_addr, &VID, &PID);
  
  // If this is a Xbox 360 Wireless controller we need to wait for a connection packet
  // on the in pipe before setting LEDs etc. So just start getting data until a controller is connected.
  if (xinput_itf->type == XBOX360_WIRELESS && xinput_itf->connected == false) {
    tuh_xinput_receive_report(dev_addr, instance);
    return;
  }
  tuh_xinput_set_led(dev_addr, instance, 0, true);
  tuh_xinput_set_led(dev_addr, instance, 1, true);
  tuh_xinput_set_rumble(dev_addr, instance, 0, 0, true);
  tuh_xinput_receive_report(dev_addr, instance);
  mounted_dev = dev_addr;
  mounted_instance = instance;
  mounted_is_xinput = true;
}

void tuh_xinput_umount_cb(uint8_t dev_addr, uint8_t instance) {
  set_led(LOW);
  mounted_dev = 0;
}


const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = *driver_count + 1;
  return xid_get_driver();
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  bool ret = false;
  ret |= xid_get_driver()->control_xfer_cb(rhport, stage, request);
  return ret;
}
