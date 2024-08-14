#include <stdint.h>

#define NUM_ENDPOINTS 2

constexpr uint8_t one_zero_byte[] = {0x00};
constexpr uint8_t one_max_byte[] = {0xFF};
constexpr uint8_t two_zero_byte[] = {0x00, 0x00};
constexpr uint8_t four_zero_byte[] = {0x00, 0x00, 0x00, 0x00};

// Reuse these to save SRAM
constexpr char manufacturer[] = "iSilentLLC";
constexpr char doorbell_model[] = "Doorbell";
constexpr char temp_model[] = "Temp & Humidity";

attribute BuildStringAtt(uint16_t a_id, char *value, uint8_t size, uint8_t a_type)
{
    uint8_t *value_t = (uint8_t *)value;
    return attribute(a_id, value_t, size, a_type, 0x01);
}

attribute manuf_attr = BuildStringAtt(MANUFACTURER_ATTR, const_cast<char *>(manufacturer), sizeof(manufacturer), ZCL_CHAR_STR);
attribute doorbell_model_attr = BuildStringAtt(MODEL_ATTR, const_cast<char *>(doorbell_model), sizeof(doorbell_model), ZCL_CHAR_STR);
attribute temp_model_attr = BuildStringAtt(MODEL_ATTR, const_cast<char *>(temp_model), sizeof(temp_model), ZCL_CHAR_STR);

attribute door_basic_attr[]{
    manuf_attr,
    doorbell_model_attr};

attribute temp_basic_attr[]{
    manuf_attr,
    temp_model_attr};

attribute door_attr[] = {
    {BINARY_PV_ATTR, const_cast<uint8_t *>(one_zero_byte), 1, ZCL_BOOL},   // present value
    {BINARY_STATUS_FLG, const_cast<uint8_t *>(one_zero_byte), 1, ZCL_MAP8} // Status flags
};

static attribute temp_attr[] = {{CURRENT_STATE, const_cast<uint8_t *>(two_zero_byte), 2, ZCL_INT16_T}};
static attribute humid_attr[] = {{CURRENT_STATE, const_cast<uint8_t *>(two_zero_byte), 2, ZCL_UINT16_T}};

static Cluster door_in_clusters[] = {
    Cluster(BASIC_CLUSTER_ID, door_basic_attr, sizeof(door_basic_attr) / sizeof(*door_basic_attr)),
    Cluster(BINARY_INPUT_CLUSTER_ID, door_attr, sizeof(door_attr) / sizeof(*door_attr))};

static Cluster t_in_clusters[] = {
    Cluster(BASIC_CLUSTER_ID, temp_basic_attr, sizeof(temp_basic_attr) / sizeof(*temp_basic_attr)),
    Cluster(TEMP_CLUSTER_ID, temp_attr, sizeof(temp_attr) / sizeof(*temp_attr)),
    Cluster(HUMIDITY_CLUSTER_ID, humid_attr, sizeof(humid_attr) / sizeof(*humid_attr))};

static Cluster out_clusters[] = {};

static Endpoint ENDPOINTS[NUM_ENDPOINTS] = {
    Endpoint(1, ON_OFF_SENSOR, door_in_clusters, out_clusters, sizeof(door_in_clusters) / sizeof(*door_in_clusters), 0),
    Endpoint(2, TEMPERATURE_SENSOR, t_in_clusters, out_clusters, sizeof(t_in_clusters) / sizeof(*t_in_clusters), 0),
};
