#include <stdint.h>

#define NUM_ENDPOINTS 2

static uint8_t *manuf = (uint8_t *)"iSilentLLC";
static attribute door_basic_attr[]{
    {MANUFACTURER_ATTR, manuf, 10, ZCL_CHAR_STR},
    {MODEL_ATTR, (uint8_t *)"DoorBell", 8, ZCL_CHAR_STR}};
static attribute temp_basic_attr[]{
    {MANUFACTURER_ATTR, manuf, 10, ZCL_CHAR_STR},
    {MODEL_ATTR, (uint8_t *)"Temp", 4, ZCL_CHAR_STR}};
static attribute door_attr[] = {
    {BINARY_PV_ATTR, 0x00, 1, ZCL_BOOL}, // present value
    {BINARY_STATUS_FLG, 0x0, 1, ZCL_MAP8}   // Status flags
};
static attribute temp_attr[] = {{CURRENT_STATE, 0x00, 2, ZCL_INT16_T}};
static attribute humid_attr[] = {{CURRENT_STATE, 0x00, 2, ZCL_UINT16_T}};

static Cluster door_in_clusters[] = {
    Cluster(BASIC_CLUSTER_ID, door_basic_attr, sizeof(door_basic_attr)),
    Cluster(BINARY_INPUT_CLUSTER_ID, door_attr, sizeof(door_attr))};
static Cluster t_in_clusters[] = {
    Cluster(BASIC_CLUSTER_ID, temp_basic_attr, sizeof(temp_basic_attr)),
    Cluster(TEMP_CLUSTER_ID, temp_attr, sizeof(temp_attr)),
    Cluster(HUMIDITY_CLUSTER_ID, humid_attr, sizeof(humid_attr))};

static Cluster out_clusters[] = {};
static Endpoint ENDPOINTS[NUM_ENDPOINTS] = {
    Endpoint(1, ON_OFF_SENSOR, door_in_clusters, out_clusters, sizeof(door_in_clusters), 0),
    Endpoint(2, TEMPERATURE_SENSOR, t_in_clusters, out_clusters, sizeof(t_in_clusters), 0),
};
