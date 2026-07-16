#include "step42_protocol.h"
#include "step42_can_transport.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

static uint8_t step42_seq = 0U;

static int Step42_CAN_SendRaw(const uint8_t *raw, uint16_t raw_len)
{
    Step42_CAN_Packet packets[STEP42_CAN_MAX_SEGMENTS];
    FDCAN_TxHeaderTypeDef header;
    uint8_t count;
    uint8_t i;

    count = Step42_CAN_Encode(raw, raw_len, packets, STEP42_CAN_MAX_SEGMENTS);
    if (count == 0U)
    {
        return -1;
    }

    header.Identifier = STEP42_CAN_COMMAND_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    for (i = 0U; i < count; i++)
    {
        header.DataLength = packets[i].len;
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, packets[i].data) != HAL_OK)
        {
            return -1;
        }
    }
    return 0;
}

void Step42_CAN_ExampleEnableAndMove(void)
{
    uint8_t raw[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    len = Step42_BuildEnable(raw, sizeof(raw), ++step42_seq, 1U);
    (void)Step42_CAN_SendRaw(raw, len);
    len = Step42_BuildSetMode(raw, sizeof(raw), ++step42_seq, STEP42_MODE_CLOSED_POSITION);
    (void)Step42_CAN_SendRaw(raw, len);
    len = Step42_BuildSetAngle(raw, sizeof(raw), ++step42_seq, 90.0f);
    (void)Step42_CAN_SendRaw(raw, len);
}
