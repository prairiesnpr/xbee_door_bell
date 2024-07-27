#include <stdint.h>

#define NUM_ENDPOINTS 2

static uint8_t *manuf = (uint8_t *)"iSilentLLC";
static attribute door_basic_attr[]{
    {0x0004, manuf, 10, ZCL_CHAR_STR},
    {0x0005, (uint8_t *)"DoorBell", 8, ZCL_CHAR_STR}};
static attribute temp_basic_attr[]{
    {0x0004, manuf, 10, ZCL_CHAR_STR},
    {0x0005, (uint8_t *)"Temp", 4, ZCL_CHAR_STR}};
static attribute door_attr[] = {
    {0x0055, 0x00, 1, ZCL_BOOL}, // present value
    {0x006F, 0x0, 1, ZCL_MAP8}   // Status flags
};
static attribute temp_attr[] = {{0x0000, 0x00, 2, ZCL_UINT16_T}};
static attribute humid_attr[] = {{0x0000, 0x00, 2, ZCL_UINT16_T}};

static Cluster door_in_clusters[] = {
    Cluster(BASIC_CLUSTER_ID, door_basic_attr, 2),
    Cluster(BINARY_INPUT_CLUSTER_ID, door_attr, 1)};
static Cluster t_in_clusters[] = {
    Cluster(BASIC_CLUSTER_ID, temp_basic_attr, 2),
    Cluster(TEMP_CLUSTER_ID, temp_attr, 1),
    Cluster(HUMIDITY_CLUSTER_ID, humid_attr, 1)};

static Cluster out_clusters[] = {};
static Endpoint ENDPOINTS[NUM_ENDPOINTS] = {
    Endpoint(1, ON_OFF_SENSOR, door_in_clusters, out_clusters, 2, 0),
    Endpoint(2, TEMPERATURE_SENSOR, t_in_clusters, out_clusters, 3, 0),
};
