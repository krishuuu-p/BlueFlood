#include <inttypes.h>
#include <string.h>
#include <random.h>
#include "contiki.h"
#include "dev/leds.h"
#include "simple-uart.h"
#include "nrf-radio-driver.h"
#include "watchdog.h"

#include "uuids.h"
#include "ble-beacon-header.h"
#include "encode-decode-hamming-crc24.h"

#include "testbed.h"
#include "nrf-gpio.h"


/*---------------------------------------------------------------------------*/
#ifndef NTX
#define ROUND_LEN (4U)
#define ROUND_RX_LEN (TESTBED_DIAMETER*ROUND_LEN)
#else
#define ROUND_LEN ((NTX))
#define ROUND_RX_LEN (TESTBED_DIAMETER*ROUND_LEN)
#endif /* NTX */
#ifndef ROUND_PERIOD_CONF_US
#error "Define round period!"
#endif
#define ROUND_LEN_MAX (ROUND_RX_LEN+ROUND_LEN)
#define ROUND_PERIOD_CONF_RTICKS US_TO_RTIMERTICKS(ROUND_PERIOD_CONF_US)
#define GLOSSY_DURATION_MAX (ROUND_LEN_MAX * SLOT_LEN)
#define ROUND_PERIOD (ROUND_PERIOD_CONF_RTICKS - ROUND_LEN_MAX * SLOT_LEN)
#define GLOSSY_ROUNDS 10
#define LEVEL_SHARING_ROUND 15



#define MINT_N_TX 10
#define CHAIN_LENGTH            NUM_NODES
#define MINT_RX_LEN ((MINT_N_TX*MINT_TESTBED_DIAMETER - 3)*(CHAIN_LENGTH))
#define MINT_LEN_MAX (MINT_RX_LEN + CHAIN_LENGTH)
#define MINT_DURATION_MAX (MINT_LEN_MAX * MINT_SLOT_LEN)
/*---------------------------------------------------------------------------*/
#if TESTBED!=WIRED_TESTBED
  #define LOG_STATE_SIZE (ROUND_LEN_MAX)
#else
  #define LOG_STATE_SIZE (ROUND_LEN_MAX)
#endif /* TESTBED!=WIRED_TESTBED */
/*---------------------------------------------------------------------------*/
#define IBEACON_SIZE  (sizeof(ble_beacon_t))
#define MINT_IBEACON_SIZE (sizeof(mint_ble_beacon_t))
#define BLUETOOTH_BEACON_PDU(S) (8+(S))
#define MINT_PACKET_AIR_TIME_MIN (PACKET_AIR_TIME(BLUETOOTH_BEACON_PDU(MINT_IBEACON_SIZE),RADIO_MODE_CONF))
#define PACKET_AIR_TIME_MIN (PACKET_AIR_TIME(BLUETOOTH_BEACON_PDU(IBEACON_SIZE),RADIO_MODE_CONF))
#define PAYLOAD_AIR_TIME_MIN (PACKET_AIR_TIME(IBEACON_SIZE,RADIO_MODE_CONF))
#define MINT_PAYLOAD_AIR_TIME_MIN (PACKET_AIR_TIME(MINT_IBEACON_SIZE,RADIO_MODE_CONF))
#define RX_SLOT_LEN (SLOT_PROCESSING_TIME+TX_CHAIN_DELAY+ US_TO_RTIMERTICKS(MY_RADIO_RAMPUP_TIME_US) + PACKET_AIR_TIME_MIN)
#define MINT_RX_SLOT_LEN (SLOT_PROCESSING_TIME+TX_CHAIN_DELAY+ US_TO_RTIMERTICKS(MY_RADIO_RAMPUP_TIME_US) + MINT_PACKET_AIR_TIME_MIN)
#define SLOT_LEN (RX_SLOT_LEN+2*GUARD_TIME_SHORT)
#define MINT_SLOT_LEN (MINT_RX_SLOT_LEN+2*GUARD_TIME_SHORT)
#define SLOT_LEN_NOTSYNCED (RX_SLOT_LEN+GUARD_TIME)
#define FIRST_SLOT_OFFSET (SLOT_PROCESSING_TIME + GUARD_TIME + ADDRESS_EVENT_T_TX_OFFSET)
/*---------------------------------------------------------------------------*/

static uint32_t testbed_ids[] = TESTBED_IDS;
static uint8_t testbed_pi_ids[] = TESTBED_PI_IDS;
extern uint16_t round, slot, logslot, join_round , sync_slot;
extern uint8_t relay,relay_min;
extern volatile rtimer_clock_t tt, t_start_round;
enum {MSG_TURN_BROADCAST=0xff, MSG_TURN_NONE=0xfe};
/*---------------------------------------------------------------------------*/
#if ROUND_ROBIN_INITIATOR
volatile uint8_t initiator_node_index = INITATOR_NODE_INDEX;
#define tx_node_id        (TESTBED_IDS[initiator_node_index])
#else
#define tx_node_id        (TESTBED_IDS[INITATOR_NODE_INDEX])
#endif /* ROUND_ROBIN_INITIATOR */
#define mint_tx_node_id (TESTBED_IDS[MINT_INITATOR_NODE_INDEX])
#define IS_INITIATOR() (my_id == tx_node_id)
#define IS_MINT_INITIATOR() (my_id == mint_tx_node_id)
/*---------------------------------------------------------------------------*/


enum app_state {
    STATE_TX_NOT_END,
    STATE_RX_NOT_END,
    STATE_MINT_STOP
};




#define APP_CHAIN_CNT_FIELD        app_packet.chain_cnt
//#define APP_RELAY_CNT_FIELD        app_packet.relay_cnt
#define APP_DATA_FIELD             app_packet.data

#define BLE_MIN_POWER (256-40)
#define BLE_MAX_POWER 8

//void print_app_states(void);
//void poll_minicast(void);
void app_start();
void app_level_sharing(uint8_t forwarded_data, uint16_t round);
void app_new_opt_start(uint8_t forwarded_data, uint16_t round);


#define print_extra true
#define PRINT_LEVEL_SHARING_DEBUG false
#define minicast_new true
#define odd_set_initiator true


//static uint8_t check_timer_miss(rtimer_clock_t ref_time, rtimer_clock_t offset, rtimer_clock_t now);

//PROCESS_NAME(mint_tx_process);

//static int get_testbed_index(uint32_t my_id, const uint32_t *testbed_ids, uint8_t testbed_size);
