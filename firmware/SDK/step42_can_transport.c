#include "step42_can_transport.h"
#include <string.h>

#define STEP42_CAN_SEGMENT_START    (0xA0U)
#define STEP42_CAN_SEGMENT_CONTINUE (0xB0U)

uint8_t Step42_CAN_Encode(const uint8_t *frame,
                          uint16_t frame_len,
                          Step42_CAN_Packet *packets,
                          uint8_t max_packets)
{
    uint16_t offset = 0U;
    uint8_t count = 0U;
    uint8_t chunk;

    if ((frame == 0) || (packets == 0) || (frame_len < 6U) ||
        (frame_len > STEP42_MAX_FRAME_SIZE) || (max_packets == 0U))
    {
        return 0U;
    }

    packets[count].data[0] = STEP42_CAN_SEGMENT_START;
    packets[count].data[1] = (uint8_t)frame_len;
    chunk = (frame_len > 6U) ? 6U : (uint8_t)frame_len;
    memcpy(&packets[count].data[2], frame, chunk);
    packets[count].len = (uint8_t)(chunk + 2U);
    count++;
    offset = chunk;

    while (offset < frame_len)
    {
        if (count >= max_packets)
        {
            return 0U;
        }
        chunk = (((uint16_t)(frame_len - offset)) > 7U) ?
                7U : (uint8_t)(frame_len - offset);
        packets[count].data[0] = (uint8_t)(STEP42_CAN_SEGMENT_CONTINUE | count);
        memcpy(&packets[count].data[1], &frame[offset], chunk);
        packets[count].len = (uint8_t)(chunk + 1U);
        offset += chunk;
        count++;
    }

    return count;
}

void Step42_CAN_ReassemblerInit(Step42_CAN_Reassembler *state)
{
    if (state != 0)
    {
        memset(state, 0, sizeof(*state));
    }
}

int Step42_CAN_ReassemblerPush(Step42_CAN_Reassembler *state,
                               const uint8_t *packet,
                               uint8_t packet_len,
                               uint8_t *frame_out,
                               uint16_t frame_out_size,
                               uint16_t *frame_len)
{
    uint8_t required;

    if ((state == 0) || (packet == 0) || (packet_len == 0U) ||
        (frame_out == 0) || (frame_len == 0))
    {
        return -1;
    }

    if (packet[0] == STEP42_CAN_SEGMENT_START)
    {
        if (packet_len < 3U || packet[1] < 6U || packet[1] > STEP42_MAX_FRAME_SIZE)
        {
            Step42_CAN_ReassemblerInit(state);
            return -1;
        }
        required = (packet[1] > 6U) ? 6U : packet[1];
        if (packet_len != (uint8_t)(required + 2U))
        {
            Step42_CAN_ReassemblerInit(state);
            return -1;
        }
        state->expected_len = packet[1];
        state->count = required;
        state->next_index = 1U;
        memcpy(state->data, &packet[2], required);
    }
    else
    {
        if (((packet[0] & 0xF0U) != STEP42_CAN_SEGMENT_CONTINUE) ||
            (state->expected_len == 0U) ||
            ((packet[0] & 0x0FU) != state->next_index))
        {
            Step42_CAN_ReassemblerInit(state);
            return -1;
        }
        required = (uint8_t)(state->expected_len - state->count);
        if (required > 7U)
        {
            required = 7U;
        }
        if (packet_len != (uint8_t)(required + 1U))
        {
            Step42_CAN_ReassemblerInit(state);
            return -1;
        }
        memcpy(&state->data[state->count], &packet[1], required);
        state->count = (uint8_t)(state->count + required);
        state->next_index++;
    }

    if (state->count != state->expected_len)
    {
        return 0;
    }
    if (frame_out_size < state->expected_len)
    {
        Step42_CAN_ReassemblerInit(state);
        return -1;
    }

    memcpy(frame_out, state->data, state->expected_len);
    *frame_len = state->expected_len;
    Step42_CAN_ReassemblerInit(state);
    return 1;
}
