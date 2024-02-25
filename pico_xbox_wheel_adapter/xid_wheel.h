#ifndef XID_WHEEL_H_
#define XID_WHEEL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <tusb.h>

#define TUD_XID_WHEEL_DESC_LEN  (9+7+7)

/* Digital Button Masks */
#define XID_DUP (1 << 0)
#define XID_DDOWN (1 << 1)
#define XID_DLEFT (1 << 2)
#define XID_DRIGHT (1 << 3)
#define XID_START (1 << 4)
#define XID_BACK (1 << 5)
#define XID_LS (1 << 6)
#define XID_RS (1 << 7)

typedef struct __attribute__((packed))
{
    uint8_t zero;
    uint8_t bLength;
    uint8_t dButtons;
    uint8_t reserved;
    uint8_t A;
    uint8_t B;
    uint8_t X;
    uint8_t Y;
    uint8_t BLACK;
    uint8_t WHITE;
    uint8_t L;
    uint8_t R;
    int16_t leftStickX;
    int16_t leftStickY;
    int16_t rightStickX;
    int16_t rightStickY;
} USB_XboxWheel_InReport_t;

typedef struct __attribute__((packed))
{
    uint8_t zero;
    uint8_t bLength;
    uint16_t lValue;
    uint16_t rValue;
} USB_XboxWheel_OutReport_t;

#define TUD_XID_WHEEL_DESC_LEN  (9+7+7)

#define TUD_XID_WHEEL_DESCRIPTOR(_itfnum, _epout, _epin) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, XID_INTERFACE_CLASS, XID_INTERFACE_SUBCLASS, 0x00, 0x00,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(32), 4, \
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(32), 4

static const uint8_t WHEEL_DESC_XID[] = {
    0x10, // Length
    0x42, // Descriptor Type
    0x00, 0x01, // BCD XID
    0x01, // Type: GameController
    0x10, // SubType: Steering wheel
    0x14, // Max Input Report Size
    0x06, // Max Output Report Size
    0xFF, 0xFF, // Alternate ID 0
    0xFF, 0xFF, // Alternate ID 1
    0xFF, 0xFF, // Alternate ID 2
    0xFF, 0xFF  // Alternate ID 3
};

static const uint8_t WHEEL_CAPABILITIES_IN[] = {
    0x00, // ReportID
    0x14, // Length
    0xFF, // Digital Buttons (RightStick,LeftStick,Back,Start,Right,Left,Down,Up)
    0x00, // Lightgun Flags
    0xFF, // A
    0xFF, // B
    0xFF, // X
    0xFF, // Y
    0xFF, // Black
    0xFF, // White
    0xFF, // Left Trigger
    0xFF, // Right Trigger
    0xFF, 0xFF, // Left Stick X
    0xFF, 0xFF, // Left Stick Y
    0xFF, 0xFF, // Right Stick X
    0xFF, 0xFF  // Right Stick X
};

static const uint8_t WHEEL_CAPABILITIES_OUT[] = {
    0x00, // ReportID
    0x06, // Length
    0xFF, 0xFF, //Left Actuator Strength
    0xFF, 0xFF  //Right Actuator Strength
};

#ifdef __cplusplus
}
#endif

#endif
