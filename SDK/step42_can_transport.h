#ifndef STEP42_CAN_TRANSPORT_H
#define STEP42_CAN_TRANSPORT_H

#include "step42_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STEP42_CAN_COMMAND_BASE     (0x300U)
#define STEP42_CAN_RESPONSE_BASE    (0x340U)
#define STEP42_CAN_NODE_ID_MIN      (1U)
#define STEP42_CAN_NODE_ID_MAX      (31U)
#define STEP42_CAN_COMMAND_ID(node_id) \
    ((uint32_t)(STEP42_CAN_COMMAND_BASE + (uint32_t)(node_id)))
#define STEP42_CAN_RESPONSE_ID(node_id) \
    ((uint32_t)(STEP42_CAN_RESPONSE_BASE + (uint32_t)(node_id)))
#define STEP42_CAN_MAX_DATA_LEN     (8U)
#define STEP42_CAN_MAX_SEGMENTS     (12U)

typedef struct
{
    uint8_t len;
    uint8_t data[STEP42_CAN_MAX_DATA_LEN];
} Step42_CAN_Packet;

typedef struct
{
    uint8_t data[STEP42_MAX_FRAME_SIZE];
    uint8_t expected_len;
    uint8_t count;
    uint8_t next_index;
} Step42_CAN_Reassembler;

/* Split one unchanged Step42 binary frame into classic-CAN packets. */
uint8_t Step42_CAN_Encode(const uint8_t *frame,
                          uint16_t frame_len,
                          Step42_CAN_Packet *packets,
                          uint8_t max_packets);

void Step42_CAN_ReassemblerInit(Step42_CAN_Reassembler *state);

/* Returns 1 with a complete raw Step42 frame, 0 while waiting, or -1 on error. */
int Step42_CAN_ReassemblerPush(Step42_CAN_Reassembler *state,
                               const uint8_t *packet,
                               uint8_t packet_len,
                               uint8_t *frame_out,
                               uint16_t frame_out_size,
                               uint16_t *frame_len);

#ifdef __cplusplus
}
#endif

#endif
