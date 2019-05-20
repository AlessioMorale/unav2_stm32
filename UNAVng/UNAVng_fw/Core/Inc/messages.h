#include <main.h>
#include <std_msgs/UInt32.h>
#pragma once
typedef enum
{
    MessageType_NONE = 0,
    MessageType_outboudn_ack = 1,
    MessageType_outbound_PIDState = 10,
    MessageType_outbound_JointState = 11,
    MessageType_inbound_JointCommand = 40,
    MessageType_inbound_BridgeConfig = 60,
    MessageType_inbound_EncoderConfig = 61,
    MessageType_inbound_LimitsConfig = 62,
    MessageType_inbound_MechanicalConfig = 63,
    MessageType_inbound_OperationConfig = 64,
    MessageType_inbound_PIDConfig = 65,
    MessageType_inbound_SafetyConfig = 66
} message_types_t;

typedef struct _pidstate_content
{
    message_types_t type;
    float output[MOTORS_COUNT];
    float error[MOTORS_COUNT];
    float p_term[MOTORS_COUNT];
    float i_term[MOTORS_COUNT];
    float d_term[MOTORS_COUNT];
    float i_min[MOTORS_COUNT];
    float i_max[MOTORS_COUNT];
    float timestep[MOTORS_COUNT];
} pidstate_content_t;

typedef struct _jointstate_content
{
    uint32_t type;
    float vel[MOTORS_COUNT];
    float pos[MOTORS_COUNT];
    float eff[MOTORS_COUNT];
} jointstate_content_t;

typedef struct _ack_content
{
    uint32_t type;
    uint32_t transactionId;
} ack_content_t;

typedef struct _generic_outbound_message
{
    union {
        uint32_t type;
        ack_content_t ackcontent;
        pidstate_content_t pidstate;
        jointstate_content_t jointstate;
    } payload;
} outbound_message_t;


