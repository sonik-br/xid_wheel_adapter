#include "xid_driver.h"

bool wheel_control_xfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request, xid_interface_t *p_xid)
{
    if (request->bmRequestType == 0xC1 && request->bRequest == 0x06 && request->wValue == 0x4200)
    {
        if (stage == CONTROL_STAGE_SETUP)
        {
            TU_LOG1("Sending WHEEL_DESC_XID\n");
            tud_control_xfer(rhport, request, (void *)WHEEL_DESC_XID, sizeof(WHEEL_DESC_XID));
        }
        return true;
    }
    else if (request->bmRequestType == 0xC1 && request->bRequest == 0x01 && request->wValue == 0x0100)
    {
        if (stage == CONTROL_STAGE_SETUP)
        {
            TU_LOG1("Sending WHEEL_CAPABILITIES_IN\n");
            tud_control_xfer(rhport, request, (void *)WHEEL_CAPABILITIES_IN, sizeof(WHEEL_CAPABILITIES_IN));
        }
        return true;
    }
    else if (request->bmRequestType == 0xC1 && request->bRequest == 0x01 && request->wValue == 0x0200)
    {
        if (stage == CONTROL_STAGE_SETUP)
        {
            TU_LOG1("Sending WHEEL_CAPABILITIES_OUT\n");
            tud_control_xfer(rhport, request, (void *)WHEEL_CAPABILITIES_OUT, sizeof(WHEEL_CAPABILITIES_OUT));
        }
        return true;
    }
    return false;
}
