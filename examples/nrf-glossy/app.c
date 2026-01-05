#include "app.h"

#define MAX_NEW 200
#define MAX_NEW_BIG 3000
static uint8_t app_rx_cnt, app_tx_cnt, app_tx_max, app_rx_cnt_main;
static mint_ble_beacon_t app_packet;
static uint8_t app_data_len, app_packet_len;
// static uint8_t app_rx_relay_cnt_last;
static volatile uint8_t app_state;
static uint32_t my_id, app_node_pos;
static uint8_t tx_buffer[255] = {0};
static uint8_t rx_buffer[255] = {0};
static uint8_t mint_tx_status[MAX_NEW_BIG] = {0};
static volatile uint8_t app_state;
static rtimer_clock_t app_t_stop, app_t_start;
volatile static rtimer_clock_t mint_tt = 0, mint_start_round = 0;
static uint32_t guard_time;
static uint16_t mint_round, mint_slot;
static uint16_t seq_counter, time_counter;
static rtimer_clock_t radio_on_time;

static int last_tx_relay_cnt, error, total, rssi_backup, received, crc_not_ok, no_address_event, address_event, wrong_address, wrong_packet;
static uint8_t app_chain_cnt, app_chain_len, app_pos, app_num_nodes, app_num_tx, app_n_tx, slot_terminator, one_subslot_packet_received_flag;
static uint16_t app_chain_cnt_no_address;
static uint8_t power_array[MAX_NEW];
static uint8_t app_data_storage[MAX_NEW];
static uint16_t app_chain_count[MAX_NEW] = {0};
static uint16_t app_chain_count_no_address[MAX_NEW] = {0};
static uint8_t app_relay_storage[MAX_NEW];
static uint8_t app_data_bitmap[MAX_NEW];
static uint8_t slot_counter;
static uint8_t round_counter;
static uint8_t round_storage[MAX_NEW];
static uint8_t chain_count_counter, chain_count_no_address_counter;
static uint8_t app_chain_storage[MAX_NEW];
static rtimer_clock_t app_time_storage[MAX_NEW], start_storage[MAX_NEW];
static uint8_t tx_cnts[MAX_NEW];
static uint8_t ntx_recieve[MAX_NEW];
static int8_t app_rssi[MAX_NEW];

static rtimer_clock_t app_start_time, prev_backup, prev_backup_new, app_end_time;
static unsigned int app_current_itr = 0;
static unsigned int data_counter = 0;
static uint8_t app_state_backup, app_data_field_backup, ntx_recieve_backup;
static uint8_t app_chain_cnt_field_backup, app_data;
static int8_t app_rssi_field_backup;
unsigned long time_target_2;
unsigned long time_target, time_mint_start_round_, time_mint_start_round;

static uint8_t my_level;
static uint16_t my_initiator_flag = 0;
static uint16_t my_round;
#define SIZE 50
uint8_t level_storage[SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t odd_chain, even_chain;
static uint8_t odd_chain_app_pos_list[SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t odd_chain_list[SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t even_chain_app_pos_list[SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t even_chain_list[SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#if PRINT_LEVEL_SHARING_DEBUG
// Debug arrays to track received values during level sharing
#define DBG_LS_SIZE 10
static uint8_t dbg_ls_slot[DBG_LS_SIZE];      // slot number
static uint16_t dbg_ls_chain_cnt[DBG_LS_SIZE]; // received chain_cnt (raw 16-bit)
static uint8_t dbg_ls_data[DBG_LS_SIZE];      // received data[0]
static uint8_t dbg_ls_stored[DBG_LS_SIZE];    // whether data was stored (1) or not (0)
static uint8_t dbg_ls_raw_byte2[DBG_LS_SIZE]; // raw byte[2] from rx_buffer for each packet
static uint8_t dbg_ls_raw_byte3[DBG_LS_SIZE]; // raw byte[3] from rx_buffer for each packet
static uint8_t dbg_ls_idx = 0;

// Debug for TX during level sharing
static uint8_t dbg_tx_slot = 255;     // slot where this node transmitted
static uint16_t dbg_tx_chain_cnt = 0; // chain_cnt that was set in packet
static uint8_t dbg_tx_data = 0;       // data that was transmitted
static uint8_t dbg_tx_raw[6] = {0};   // raw first 6 bytes of TX packet

// Debug for RX raw bytes (first received packet only)
static uint8_t dbg_rx_raw[6] = {0};   // raw first 6 bytes of first RX packet
static uint8_t dbg_rx_raw_captured = 0;
static uint8_t dbg_rx_raw_slot = 255; // slot number when RX_RAW was captured
#endif /* PRINT_LEVEL_SHARING_DEBUG */

static uint8_t app_data_odd_storage[MAX_NEW];
;
static uint32_t app_node_pos_optimal;
static uint8_t power_array_odd[MAX_NEW], ntx_itr;

static uint8_t
check_timer_miss(rtimer_clock_t ref_time, rtimer_clock_t offset, rtimer_clock_t now)
{
	rtimer_clock_t target = ref_time + offset;
	uint8_t now_has_overflowed = now < ref_time;
	uint8_t target_has_overflowed = target < ref_time;

	if (now_has_overflowed == target_has_overflowed)
	{
		/* Both or none have overflowed, just compare now to the target */
		return target <= now;
	}
	else
	{
		/* Either now or target of overflowed.
		 * If it is now, then it has passed the target.
		 * If it is target, then we haven't reached it yet.
		 *  */
		return now_has_overflowed;
	}
}

static int get_testbed_index(uint32_t my_id, const uint32_t *testbed_ids, uint8_t testbed_size)
{
	int i;
	for (i = 0; i < testbed_size; i++)
	{
		if (my_id == testbed_ids[i])
		{
			return i;
		}
	}
	return -1;
}

static uint8_t check_initiator_flag(uint8_t l)
{
	// odd transmitting first
#if odd_set_initiator
	if (l % 2 != 0)
	{
		return 1;
	}
	else
		return 0;
#else
	// even transmitting first
	if (l % 2 != 0)
	{
		return 0;
	}
	else
		return 1;
#endif
}

static int get_odd_list_index(uint32_t my_id)
{
	int i;

#if odd_set_initiator
	for (i = 0; i < odd_chain; i++)
	{
		if (my_id == odd_chain_list[i])
		{
			return i;
		}
	}
	return -1;
#else
	for (i = 0; i < even_chain; i++)
	{
		if (my_id == even_chain_list[i])
		{
			return i;
		}
	}
	return -1;
#endif
}
void sort_generated_list()
{

	uint8_t i, j;
#if odd_set_initiator
	for (i = 0; i < odd_chain; ++i)
	{
		for (j = i + 1; j < odd_chain; ++j)
		{
			if (odd_chain_list[i] > odd_chain_list[j])
			{
				uint8_t a = odd_chain_list[i];
				odd_chain_list[i] = odd_chain_list[j];
				odd_chain_list[j] = a;

				a = odd_chain_app_pos_list[i];
				odd_chain_app_pos_list[i] = odd_chain_app_pos_list[j];
				odd_chain_app_pos_list[j] = a;
			}
		}
	}
#else
	for (i = 0; i < even_chain; ++i)
	{
		for (j = i + 1; j < even_chain; ++j)
		{
			if (even_chain_list[i] > even_chain_list[j])
			{
				uint8_t a = even_chain_list[i];
				even_chain_list[i] = even_chain_list[j];
				even_chain_list[j] = a;

				a = even_chain_app_pos_list[i];
				even_chain_app_pos_list[i] = even_chain_app_pos_list[j];
				even_chain_app_pos_list[j] = a;
			}
		}
	}
#endif
}
void store_levels()
{
	uint8_t i;
	even_chain = 0;
	odd_chain = 0;
	for (i = 0; i < app_chain_len; i++)
	{
		if (app_data_storage[i] != 0)
		{
			level_storage[i] = app_data_storage[i];
		}
#if odd_set_initiator
		if (level_storage[i] != 0 && level_storage[i] % 2 != 0)
		{
			odd_chain_list[odd_chain] = testbed_pi_ids[i];
			odd_chain_app_pos_list[odd_chain++] = i;
		}
#else
		if (level_storage[i] != 0 && level_storage[i] % 2 == 0)
		{
			even_chain_list[even_chain] = testbed_pi_ids[i];
			even_chain_app_pos_list[even_chain++] = i;
		}
#endif
	}

	// sort_generated_list();
}

void print_app_states()
{
	printf("App States:\n");
	uint16_t i, j;
	printf("my id-%2u my_round-%2d my_initiator_flag-%2u other-%2d %2d %2d\n", testbed_pi_ids[app_node_pos], my_round, my_initiator_flag, RADIO_MODE_CONF, minicast_new, BLE_MAX_POWER);
	printf("{round-%d} app_packet_len-%2d \n", round, app_packet_len);
	printf("some params- %5lu %lu %lu %lu %lu %lu %lu %lu %lu %lu\n", (unsigned long)MINT_RX_SLOT_LEN, (unsigned long)MINT_PACKET_AIR_TIME_MIN, (unsigned long)GUARD_TIME_SHORT, (unsigned long)MINT_SLOT_LEN, (unsigned long)SLOT_PROCESSING_TIME, (unsigned long)TX_CHAIN_DELAY, (unsigned long)MINT_PACKET_AIR_TIME_MIN, (unsigned long)US_TO_RTIMERTICKS(MY_RADIO_RAMPUP_TIME_US), (unsigned long)SLOT_PROCESSING_TIME_PKT_END, (unsigned long)SLOT_PROCESSING_TIME_PKT_START);
#if print_extra
	printf("{mtx-%d} ", my_round);
	for (i = 0; i < seq_counter && i < MAX_NEW_BIG; i++)
	{
		printf("%c", mint_tx_status[i]);
	}
	printf("\n");
#endif
	i = 0;
	j = 0;

	// printf("| ");
	printf("even_chain-%5u	odd_chain-%5u\n", even_chain, odd_chain);
	// printf("| ");

	unsigned long latency = 0;
	uint8_t num_recv = 0;
	printf("level_storage: ");
	for (i = 0; i < app_chain_len; i++)
	{
		printf("%2u", level_storage[i]);
		if (app_data_storage[i] != 0)
		{
			unsigned long time_measure = (unsigned long)app_time_storage[i] * 1e6 / RTIMER_SECOND;
			num_recv++;
			if (time_measure > latency)
			{
				latency = time_measure;
			}
		}
		else
		{
			unsigned long radio_on = (unsigned long)(app_end_time - app_start_time);
			unsigned long time_measure = (unsigned long)radio_on * 1e6 / RTIMER_SECOND;
			if (time_measure > latency)
			{
				latency = time_measure;
			}
		}
	}
	// printf("| ");
	printf("\n");
	printf("%2u:%lu.%lu  ", num_recv, latency / 1000, latency % 1000);
	printf("| ");
	for (i = 0; i < app_chain_len; i++)
	{
		if (app_data_storage[i] != 0)
		{
			unsigned long time_measure = (unsigned long)app_time_storage[i] * 1e6 / RTIMER_SECOND;
			printf("%2u:%u;-[%d](%lu.%lu)  ", app_data_storage[i], ntx_recieve[i], app_rssi[i] - 45, time_measure / 1000, time_measure % 1000);
		}
		else
		{
			unsigned long radio_on = (unsigned long)(app_end_time - app_start_time);
			unsigned long radio_on_msr = (unsigned long)radio_on * 1e6 / RTIMER_SECOND;
			printf("-:-;-[-](%lu.%lu)  ", app_data_storage[i], radio_on_msr / 1000, radio_on_msr % 1000);
		}
	}
	printf("\n");
	
#if PRINT_LEVEL_SHARING_DEBUG
	// Debug: print TX info for level sharing with raw bytes
	printf("TX_DEBUG: s%u c%u d%u | raw:%02X %02X %02X %02X %02X %02X\n", 
		dbg_tx_slot, dbg_tx_chain_cnt, dbg_tx_data,
		dbg_tx_raw[0], dbg_tx_raw[1], dbg_tx_raw[2], dbg_tx_raw[3], dbg_tx_raw[4], dbg_tx_raw[5]);
	
	// Debug: print first RX raw bytes WITH SLOT NUMBER
	printf("RX_RAW: slot%u | %02X %02X %02X %02X %02X %02X\n", dbg_rx_raw_slot,
		dbg_rx_raw[0], dbg_rx_raw[1], dbg_rx_raw[2], dbg_rx_raw[3], dbg_rx_raw[4], dbg_rx_raw[5]);
	
	// Debug: print received packet info for level sharing with raw byte2/3
	printf("RX_DEBUG: %u pkts | ", dbg_ls_idx);
	for (i = 0; i < dbg_ls_idx && i < DBG_LS_SIZE; i++) {
		printf("s%u:c%u:d%u:r%02X%02X:%c ", dbg_ls_slot[i], dbg_ls_chain_cnt[i], dbg_ls_data[i], 
			dbg_ls_raw_byte2[i], dbg_ls_raw_byte3[i], dbg_ls_stored[i] ? 'Y' : 'N');
	}
	printf("\n");
#endif /* PRINT_LEVEL_SHARING_DEBUG */
}

void fill_data(uint8_t data, uint8_t len)
{
	uint8_t i;
	for (int i = 0; i < len; i++)
	{
		app_packet.data[i] = data;
	}
}

void rx_packet_data()
{

	// First Packet received
	//	if(app_rx_cnt==0){
	//		app_rx_relay_cnt_last = app_relay_cnt_field_backup;
	//		if(app_tx_cnt==0)
	//			last_tx_relay_cnt = app_relay_cnt_field_backup-1;
	//	}

	uint8_t rx_pos = app_chain_cnt_field_backup;  // Use local variable to avoid overwriting global app_pos
	// If the data is not there in the current node and the data recieved by the node in this subslot is non-zero then fdo the operation
	if (app_data_storage[rx_pos] == 0 && app_data_field_backup != 0)
	{
		// Save the data received in the data storage array
		app_data_storage[rx_pos] = app_data_field_backup;
		// Save the data received in the data chain array
		app_chain_storage[data_counter++] = app_data_field_backup;
		app_time_storage[rx_pos] = RTIMER_NOW() - app_start_time;
		//	app_relay_storage[rx_pos] = app_relay_cnt_field_backup;
		ntx_recieve[rx_pos] = ntx_recieve_backup;
		app_rssi[rx_pos] = app_rssi_field_backup;
		// Number of Data packet received
		app_rx_cnt_main++;
		// Set the power array at that pos 1 so that this node will transmit the data
		power_array[rx_pos] = 1;
	}

	// If packet received with a new relay count
	//	if (app_relay_cnt_field_backup > app_rx_relay_cnt_last){
	//		//Store the relay count
	//		app_data_bitmap[app_current_itr++] = app_rx_cnt_main;
	//		app_rx_relay_cnt_last = app_relay_cnt_field_backup;
	//		app_chain_storage[data_counter++] = 0;
	//	}
	prev_backup = RTIMER_NOW();
	rssi_backup = (int)((signed char)app_rssi_field_backup) - 45;
	app_rx_cnt++;
}

static void make_packet(mint_ble_beacon_t *pkt)
{
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
	pkt->radio_len = sizeof(mint_ble_beacon_t) - 1; /* execlude len field */
#else
	pkt->radio_len = sizeof(mint_ble_beacon_t) - 2; /* len + pdu_header */ // length of the rest of the packet

#endif
	pkt->pdu_header = 0x42; // pdu type: 0x02 ADV_NONCONN_IND, rfu 0, rx 0, tx 1 //2;
	//	pkt->adv_address_low = MY_ADV_ADDRESS_LOW;
	//	pkt->adv_address_hi = MY_ADV_ADDRESS_HI;

#if (PACKET_IBEACON_FORMAT)
	//	pkt->ad_flags_length = 2; //2bytes flags
	//	pkt->ad_flags_type = 1; //1=flags
	//	pkt->ad_flags_data = 6; //(non-connectable, undirected advertising, single-mode device)
	//	pkt->ad_length = 0x0b; //26 bytes, the remainder of the packet
	//	pkt->ad_type = 0xff; //manufacturer specific
	//	pkt->company_id = 0x004cU; //Apple ID
	//	pkt->beacon_type = 0x1502;//0x0215U; //proximity ibeacon
	//	pkt->power = 0;//256 - 60; //RSSI = -60 dBm; Measured Power = 256 – 60 = 196 = 0xC4
#endif
	uint8_t pkt_pos = APP_CHAIN_CNT_FIELD;  // Use local variable to avoid overwriting global app_pos

	if (power_array[pkt_pos])
	{
		// if(power_array[pkt_pos]) {
		my_radio_set_tx_power(BLE_MAX_POWER);
		fill_data(app_data_storage[pkt_pos], app_data_len);
	}
	else
	{
		my_radio_set_tx_power(BLE_MIN_POWER);
		fill_data(0, app_data_len);
	}
}

void initialize_storage()
{
	uint8_t i;

	for (i = 0; i < MAX_NEW; i++)
	{
		app_chain_storage[i] = 0;
		tx_cnts[i] = 0;
		app_data_bitmap[i] = 0;
		app_chain_count[i] = 0;
	}

	for (i = 0; i < app_chain_len; i++)
	{
		app_data_storage[i] = 0;
		power_array[i] = 0;
		app_time_storage[i] = 0;
		app_rssi[i] = 0;
		app_relay_storage[i] = 0;
	}
	//	power_array[0]=1;
	//	power_array[app_chain_len-1]=1;
}

void rx_new_packet_data()
{

	//	if(app_rx_cnt==0){
	//		app_rx_relay_cnt_last = app_relay_cnt_field_backup;
	//	}
	//	if (app_relay_cnt_field_backup > (app_rx_relay_cnt_last)){
	//		app_data_bitmap[app_current_itr++] = app_rx_cnt_main;
	//		app_rx_relay_cnt_last = app_relay_cnt_field_backup;
	//		app_chain_storage[data_counter++] = 0;
	//	}
	app_pos = app_chain_cnt_field_backup;
	if (app_data_storage[app_pos] == 0 && app_data_field_backup != 0)
	{
		app_data_storage[app_pos] = app_data_field_backup;
		app_time_storage[app_pos] = RTIMER_NOW() - app_start_time;
		ntx_recieve[app_pos] = ntx_recieve_backup;
		app_rssi[app_pos] = app_rssi_field_backup;

		// app_relay_storage[app_pos] = app_relay_cnt_field_backup;
		app_chain_storage[data_counter++] = app_data_field_backup;
		app_rx_cnt_main++;
		power_array[app_pos] = 1;
	}
	prev_backup = RTIMER_NOW();
}

void try_rx(uint8_t ntx_i)
{
	uint8_t got_payload_event, got_address_event, got_end_event, slot_started, last_crc_is_ok, last_rx_ok;
	got_payload_event = 0, got_address_event = 0, got_end_event = 0, slot_started = 0, last_crc_is_ok = 0, last_rx_ok = 0;
	uint8_t channel = BLE_CHANNEL_37_FREQ;
	rtimer_clock_t rx_target_time, rx_tn, rx_tref, rx_toffset, t_proc;
	uint8_t rx_missed_slot = 0;

	rx_target_time = mint_tt - ADDRESS_EVENT_T_TX_OFFSET - guard_time;
	rx_tn = RTIMER_NOW();
	rx_tref = mint_start_round - FIRST_SLOT_OFFSET;
	rx_toffset = (mint_slot)*MINT_SLOT_LEN + FIRST_SLOT_OFFSET - ADDRESS_EVENT_T_TX_OFFSET - guard_time;
	rx_missed_slot = check_timer_miss(rx_tref, rx_toffset, rx_tn);

	if (!rx_missed_slot)
	{
		// t_proc = RTIMER_NOW();
		schedule_rx_abs(rx_buffer, GET_CHANNEL(mint_round, mint_slot), rx_target_time);
		t_proc = RTIMER_NOW() - rx_tn;
		BUSYWAIT_UNTIL_ABS(NRF_TIMER0->EVENTS_COMPARE[0] != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START);
		slot_started = NRF_TIMER0->EVENTS_COMPARE[0];
		if (slot_started)
		{
			// nrf_gpio_pin_toggle(ROUND_INDICATOR_PIN);
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
			BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_FRAMESTART != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START + ADDRESS_EVENT_T_TX_OFFSET);
			got_address_event = NRF_RADIO->EVENTS_FRAMESTART;
#else
			BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START + ADDRESS_EVENT_T_TX_OFFSET);
			got_address_event = NRF_RADIO->EVENTS_ADDRESS;
#endif
		}
	}

	//}

	if (got_address_event)
	{
		total++;
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
		// no EVENTS_PAYLOAD is emitted
		//  PAYLOAD_AIR_TIME_MIN includes CRC
		BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, get_rx_ts() + PAYLOAD_AIR_TIME_MIN + SLOT_PROCESSING_TIME_PKT_END);

		got_end_event = NRF_RADIO->EVENTS_END;
		last_rx_ok = got_payload_event = got_end_event;
		last_crc_is_ok = USE_HAMMING_CODE || ((got_end_event != 0U) && (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_CRCOk));
		// last_crc_is_ok = 1; //XXX
#else
		BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_PAYLOAD != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN);
		got_payload_event = NRF_RADIO->EVENTS_PAYLOAD;
		last_rx_ok = got_payload_event;
		if (got_payload_event)
		{
			BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN + CRC_AIR_T + SLOT_PROCESSING_TIME_PKT_END);
			got_end_event = NRF_RADIO->EVENTS_END;
			last_crc_is_ok = USE_HAMMING_CODE || ((got_end_event != 0U) && (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_CRCOk));
		}
#endif /* (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit) */
	}
	/* check if it is a valid packet: a. our uuid and b. CRC ok */
	if (last_rx_ok && last_crc_is_ok)
	{
		mint_ble_beacon_t *rx_pkt = (mint_ble_beacon_t *)rx_buffer;
		// #if USE_HAMMING_CODE
		// rx_pkt = (mint_ble_beacon_t *) encode_decode_buffer;
		// last_crc_is_ok = decode_ble_packet(rx_buffer, encode_decode_buffer) == 0;
		// #endif

		/* check if it is our beacon packet */
		// last_rx_ok = last_crc_is_ok ? (( rx_pkt->adv_address_low == MY_ADV_ADDRESS_LOW ) && ( rx_pkt->adv_address_hi == MY_ADV_ADDRESS_HI )) : 0;
		//  last_rx_ok = last_crc_is_ok; //XXX!

		//		if(last_rx_ok){
		received++;
		memcpy(&app_packet, &rx_buffer, rx_pkt->radio_len + 2);
		if (APP_CHAIN_CNT_FIELD == app_chain_cnt)
		{
			app_state_backup = app_state;
			app_chain_cnt_field_backup = APP_CHAIN_CNT_FIELD;
			app_data_field_backup = APP_DATA_FIELD[0];
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
			app_rssi_field_backup = app_packet.lqi;
#else
			app_rssi_field_backup = get_radio_rssi();
#endif
			// app_relay_cnt_field_backup = APP_RELAY_CNT_FIELD;
			ntx_recieve_backup = ntx_i;

			rx_new_packet_data();
		}
		//		}
		//		else{
		//			error++;
		//		}
	}
#if print_extra
	if (seq_counter < MAX_NEW_BIG)
	{
		if (last_rx_ok && last_crc_is_ok)
		{
			mint_tx_status[seq_counter++] = '-';
		}
		else if (!slot_started)
		{
			mint_tx_status[seq_counter++] = 'M';
		}
		else if (!got_address_event)
		{
			mint_tx_status[seq_counter++] = 'A';
		}
		else if (!got_payload_event)
		{
			mint_tx_status[seq_counter++] = 'P';
		}
		else if (!got_end_event)
		{
			mint_tx_status[seq_counter++] = 'E';
		}
		else if (!last_crc_is_ok)
		{
			mint_tx_status[seq_counter++] = 'C';
		}
		else if (!last_rx_ok)
		{
			mint_tx_status[seq_counter++] = 'W'; // wrong address
		}
		else
		{
			mint_tx_status[seq_counter++] = '?';
		}
	}
#endif
}

/**
 * @brief Single-round level sharing using TDMA + Opportunistic Reception (OR)
 * 
 * Protocol: Pure TDMA (no SI - Simultaneous Initiation)
 * - All nodes participate equally (no odd/even initiator distinction)
 * - Each node transmits its level in its own TDMA slot (position-based)
 * - All other nodes listen and opportunistically receive (OR)
 * - Chain length = NUM_NODES (all nodes get a slot)
 * - Duration: ~260ms for 48 nodes (48 slots × 5.4ms/slot)
 * 
 * How it works:
 * 1. All nodes start in RX mode
 * 2. When chain position matches my position: TX my level
 * 3. At all other positions: RX and collect neighbor levels (OR)
 * 4. After all positions: Complete with full level set
 * 
 * Key differences from SI-based data dissemination:
 * - No initiator check (all nodes equal)
 * - No chain position filtering during RX (accept all positions)
 * - Single round only (no retransmissions needed in reliable testbed)
 */
void app_level_sharing(uint8_t forwarded_data, uint16_t round_)
{
#if print_extra
	if (time_counter < MAX_NEW)
		start_storage[time_counter++] = RTIMER_NOW() - t_start_round;
#endif
	my_level = forwarded_data;
	my_round = round_;
	seq_counter = 0;
	slot_counter = 0;
	chain_count_counter = 0;
	chain_count_no_address_counter = 0;
	rtimer_clock_t target = t_start_round + GLOSSY_DURATION_MAX + MINT_DURATION_MAX + GUARD_TIME;
	// rtimer_clock_t target_2 = RTIMER_NOW() + MINT_DURATION_MAX;
	// time_target_2 = (unsigned long)target_2 * 1e6 / RTIMER_SECOND;
	// time_target = (unsigned long)target * 1e6 / RTIMER_SECOND;

	my_radio_init(&my_id, tx_buffer);
	// app_node_pos = get_testbed_index(my_id,testbed_ids,TESTBED_SIZE) + 1;
	app_node_pos = get_testbed_index(my_id, testbed_ids, TESTBED_SIZE);

	app_chain_len = CHAIN_LENGTH;

	app_num_nodes = NUM_NODES;
	app_n_tx = MINT_N_TX;
	app_num_tx = 1;
	app_tx_max = app_n_tx * app_num_tx;
	// last_tx_relay_cnt = -1;
	//	app_rx_relay_cnt_last = 0;
	app_rx_cnt_main = 0;
	app_data_len = USER_DATA_LEN;
	app_data = 1;

	
	initialize_storage();

	app_start_time = RTIMER_NOW();

	if (app_node_pos >= 0 && app_node_pos < app_num_nodes)
	{
		// app_pos = ((app_node_pos-1)/(app_chain_len-2))*app_chain_len + ((app_node_pos-1)%(app_chain_len-2)) + 1;
		app_pos = app_node_pos;
		power_array[app_pos] = 1;
		app_data_storage[app_pos] = my_level;
		app_time_storage[app_pos] = RTIMER_NOW() - app_start_time;
		app_rssi[app_pos] = 0;
		app_rx_cnt_main++;
	}

	app_tx_cnt = 0;
	app_rx_cnt = 0;
	wrong_address = 0;
	address_event = 0;
	received = 0;
	crc_not_ok = 0;
	app_chain_cnt_no_address = 0;
	no_address_event = 0;
	guard_time = GUARD_TIME;
	// app_rx_cnt_other = 0;
	app_current_itr = 0;
	app_chain_cnt = 0;
	data_counter = 0;
	// app_t_start = RTIMER_NOW_DCO();
	radio_on_time = 0;
	//	APP_RELAY_CNT_FIELD = 0;
	APP_CHAIN_CNT_FIELD = 0;

	app_packet_len = sizeof(mint_ble_beacon_t);

	// TDMA+OR Level Sharing: All nodes participate equally
	// No SI (Simultaneous Initiation) - just pure TDMA
	// Each node will TX at its own position, RX at all others
	app_state = STATE_RX_NOT_END;  // Start in RX mode
	
#if PRINT_LEVEL_SHARING_DEBUG
	// Reset debug arrays for level sharing
	dbg_ls_idx = 0;
	dbg_tx_slot = 255;
	dbg_tx_chain_cnt = 0;
	dbg_tx_data = 0;
	dbg_rx_raw_captured = 0;
	dbg_rx_raw_slot = 255;
	for (int di = 0; di < 6; di++) { dbg_tx_raw[di] = 0; dbg_rx_raw[di] = 0; }
	for (int di = 0; di < DBG_LS_SIZE; di++) { dbg_ls_raw_byte2[di] = 0; dbg_ls_raw_byte3[di] = 0; }
#endif /* PRINT_LEVEL_SHARING_DEBUG */

	//	mint_start_round = t_start_round + GLOSSY_DURATION_MAX  + FIRST_SLOT_OFFSET+GUARD_TIME;
	mint_start_round = t_start_round + GLOSSY_DURATION_MAX + FIRST_SLOT_OFFSET;
	// mint_start_round_ = RTIMER_NOW()  + FIRST_SLOT_OFFSET;
	//	time_mint_start_round_ = (unsigned long)mint_start_round_ * 1e6 / RTIMER_SECOND;
	//	time_mint_start_round = (unsigned long)mint_start_round * 1e6 / RTIMER_SECOND;

	mint_round = round;
	mint_slot = 0;
	slot_terminator = 0;

#if print_extra
	mint_tx_status[seq_counter] = 'S';
	seq_counter++;
	round_storage[round_counter++] = mint_round;
#endif
	app_chain_count[chain_count_counter++] = app_chain_len;
	while (RTIMER_CLOCK_LT(RTIMER_NOW(), target))
	{
		// In TDMA mode, position should follow slot number
		app_chain_cnt = mint_slot;
		
		// Note: slot_terminator logic removed for TDMA - we iterate exactly once through all positions

#if print_extra
		// if(slot_counter < MAX_NEW_BIG)
		//	slot_storage[slot_counter++] = mint_slot;
#endif
		mint_tt = mint_start_round + (mint_slot)*MINT_SLOT_LEN;
#if print_extra
		// if(slot_counter < MAX_NEW_BIG)
		// slot_time_storage[slot_counter++] = mint_tt;
#endif
		// TDMA Logic: Check if current slot matches my position
		if (app_chain_cnt == app_pos && power_array[app_pos] == 1)
		{
			// My turn to transmit!
			{
				guard_time = GUARD_TIME_SHORT;
				APP_CHAIN_CNT_FIELD = app_chain_cnt;  // Set chain position in packet
				// printf("[TX] pos=%d, my_level=%d, app_data_storage[%d]=%d\n", app_pos, my_level, app_pos, app_data_storage[app_pos]);
				make_packet(&app_packet);
				
#if PRINT_LEVEL_SHARING_DEBUG
			// Debug: record what we're transmitting
			dbg_tx_slot = mint_slot;
			dbg_tx_chain_cnt = APP_CHAIN_CNT_FIELD;
			dbg_tx_data = APP_DATA_FIELD[0];
			
			// Debug: record raw bytes of packet for verification
			volatile uint8_t *raw = (volatile uint8_t *)&app_packet;
			dbg_tx_raw[0] = raw[0];  // pdu_header
			dbg_tx_raw[1] = raw[1];  // radio_len
			dbg_tx_raw[2] = raw[2];  // chain_cnt low byte
			dbg_tx_raw[3] = raw[3];  // chain_cnt high byte
			dbg_tx_raw[4] = raw[4];  // data[0]
			dbg_tx_raw[5] = raw[5];  // data[1]
#endif /* PRINT_LEVEL_SHARING_DEBUG */
				
				uint8_t *tx_msg = (uint8_t *)&app_packet;
				schedule_tx_abs(tx_msg, GET_CHANNEL(mint_round, mint_slot), mint_tt - ADDRESS_EVENT_T_TX_OFFSET + ARTIFICIAL_TX_OFFSET);

				BUSYWAIT_UNTIL_ABS(NRF_TIMER0->EVENTS_COMPARE[0] != 0U, mint_tt - ADDRESS_EVENT_T_TX_OFFSET + ARTIFICIAL_TX_OFFSET);
				if (!NRF_TIMER0->EVENTS_COMPARE[0])
				{
#if print_extra
					if (seq_counter < MAX_NEW_BIG)
						mint_tx_status[seq_counter++] = 'T';
#endif
				}
				else
				{
					BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, mint_tt + ARTIFICIAL_TX_OFFSET + MINT_PACKET_AIR_TIME_MIN);
					if (!NRF_RADIO->EVENTS_END)
					{
#if print_extra
						if (seq_counter < MAX_NEW_BIG)
							mint_tx_status[seq_counter++] = 'R';
#endif
					}
					else
					{
#if print_extra
						if (seq_counter < MAX_NEW_BIG)
							mint_tx_status[seq_counter++] = 'B';
#endif
					}
				}
				// After transmitting, move to next slot (mint_slot++ at end of loop)
				// app_chain_cnt is now automatically set to mint_slot at loop start
#if print_extra
				if (chain_count_counter < MAX_NEW)
					app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
				
				// Check if we've completed the full chain
				if (app_chain_cnt >= app_chain_len - 1)  // -1 because we check at start of last slot
				{
					app_state = STATE_MINT_STOP;  // Done with level sharing
				}
			}
		}
		else if (app_state == STATE_RX_NOT_END)
		{
			// Not my turn - listen for others' transmissions
			guard_time = GUARD_TIME;
			uint8_t got_payload_event, got_address_event, got_end_event, slot_started, last_crc_is_ok, last_rx_ok, got_wrong_packet;
			got_payload_event = 0, got_address_event = 0, got_end_event = 0, slot_started = 0, last_crc_is_ok = 0, last_rx_ok = 0, got_wrong_packet = 0;
			uint8_t channel = BLE_CHANNEL_37_FREQ;
			rtimer_clock_t rx_target_time, rx_tn, rx_tref, rx_toffset, t_proc;
			uint8_t rx_missed_slot = 0;
			rx_target_time = mint_tt - ADDRESS_EVENT_T_TX_OFFSET - guard_time;
			rx_tn = RTIMER_NOW();
			rx_tref = mint_start_round - FIRST_SLOT_OFFSET;
			rx_toffset = (mint_slot)*MINT_SLOT_LEN + FIRST_SLOT_OFFSET - ADDRESS_EVENT_T_TX_OFFSET - guard_time;
			rx_missed_slot = check_timer_miss(rx_tref, rx_toffset, rx_tn);
			if (!rx_missed_slot)
			{
				schedule_rx_abs(rx_buffer, GET_CHANNEL(mint_round, mint_slot), rx_target_time);
				t_proc = RTIMER_NOW() - rx_tn;
				BUSYWAIT_UNTIL_ABS(NRF_TIMER0->EVENTS_COMPARE[0] != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START);
				slot_started = NRF_TIMER0->EVENTS_COMPARE[0];
				if (slot_started)
				{
					// nrf_gpio_pin_toggle(ROUND_INDICATOR_PIN);
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
					BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_FRAMESTART != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START + ADDRESS_EVENT_T_TX_OFFSET);
					got_address_event = NRF_RADIO->EVENTS_FRAMESTART;
#else
					BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START + ADDRESS_EVENT_T_TX_OFFSET);
					got_address_event = NRF_RADIO->EVENTS_ADDRESS;
#endif
				}
			}

			if (got_address_event)
			{
				address_event++;
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
				// no EVENTS_PAYLOAD is emitted
				//  PAYLOAD_AIR_TIME_MIN includes CRC
				BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN + SLOT_PROCESSING_TIME_PKT_END);

				got_end_event = NRF_RADIO->EVENTS_END;
				last_rx_ok = got_payload_event = got_end_event;
				last_crc_is_ok = USE_HAMMING_CODE || ((got_end_event != 0U) && (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_CRCOk));
				// last_crc_is_ok = 1; //XXX
#else
				BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_PAYLOAD != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN);
				got_payload_event = NRF_RADIO->EVENTS_PAYLOAD;
				last_rx_ok = got_payload_event;
				if (got_payload_event)
				{
					BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN + CRC_AIR_T + SLOT_PROCESSING_TIME_PKT_END);
					got_end_event = NRF_RADIO->EVENTS_END;
					last_crc_is_ok = USE_HAMMING_CODE || ((got_end_event != 0U) && (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_CRCOk));
				}
#endif /* (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit) */

				/* check if it is a valid packet: a. our uuid and b. CRC ok */
				if (last_rx_ok && last_crc_is_ok)
				{
					mint_ble_beacon_t *rx_pkt = (mint_ble_beacon_t *)rx_buffer;

					/* check if it is our beacon packet */
					//	last_rx_ok = last_crc_is_ok ? (( rx_pkt->adv_address_low == MY_ADV_ADDRESS_LOW ) && ( rx_pkt->adv_address_hi == MY_ADV_ADDRESS_HI )) : 0;
					// last_rx_ok = last_crc_is_ok; //XXX!

					//					if(last_rx_ok){
					
#if PRINT_LEVEL_SHARING_DEBUG
					// Debug: capture raw RX bytes BEFORE memcpy (first valid packet only)
					if (!dbg_rx_raw_captured) {
						dbg_rx_raw[0] = rx_buffer[0];
						dbg_rx_raw[1] = rx_buffer[1];
						dbg_rx_raw[2] = rx_buffer[2];
						dbg_rx_raw[3] = rx_buffer[3];
						dbg_rx_raw[4] = rx_buffer[4];
						dbg_rx_raw[5] = rx_buffer[5];
						dbg_rx_raw_slot = mint_slot;  // Record which slot this came from
						dbg_rx_raw_captured = 1;
					}
#endif /* PRINT_LEVEL_SHARING_DEBUG */

					memcpy(&app_packet, &rx_buffer, rx_pkt->radio_len + 2);
					// For level sharing, accept packets from ANY position (OR - Opportunistic Reception)
					// No chain position check needed - we want to collect all levels
					if (1)  // Always accept during level sharing
					{
						received++;

						if (APP_CHAIN_CNT_FIELD == app_chain_len - 1)
						{
							app_state = STATE_TX_NOT_END;
						}

						app_state_backup = app_state;
						app_chain_cnt_field_backup = APP_CHAIN_CNT_FIELD;
						app_data_field_backup = APP_DATA_FIELD[0];
						
#if PRINT_LEVEL_SHARING_DEBUG
						// Debug: record received values BEFORE rx_packet_data()
						uint8_t dbg_current_idx = 255;  // invalid
						if (dbg_ls_idx < DBG_LS_SIZE) {
							dbg_current_idx = dbg_ls_idx;
							dbg_ls_slot[dbg_ls_idx] = mint_slot;
							dbg_ls_chain_cnt[dbg_ls_idx] = APP_CHAIN_CNT_FIELD;  // Raw 16-bit value
							dbg_ls_data[dbg_ls_idx] = APP_DATA_FIELD[0];
							dbg_ls_stored[dbg_ls_idx] = 0;  // Will be set to 1 by rx_packet_data if stored
							// Capture raw bytes for this specific packet
							dbg_ls_raw_byte2[dbg_ls_idx] = rx_buffer[2];
							dbg_ls_raw_byte3[dbg_ls_idx] = rx_buffer[3];
							dbg_ls_idx++;
						}
#endif /* PRINT_LEVEL_SHARING_DEBUG */
						
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
						app_rssi_field_backup = app_packet.lqi;
#else
						app_rssi_field_backup = get_radio_rssi();
#endif
						//	app_relay_cnt_field_backup = APP_RELAY_CNT_FIELD;
						one_subslot_packet_received_flag = 1;

						// Packet accepted - position already advanced by app_chain_cnt = mint_slot
						// No need to manually increment
#if print_extra
							if (chain_count_counter < MAX_NEW)
								app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
							
							// Stop if all positions covered
							if (app_chain_cnt >= app_chain_len - 1)  // -1 because we check at start of last slot
							{
								app_state = STATE_MINT_STOP;
							}

						rx_packet_data();
						
#if PRINT_LEVEL_SHARING_DEBUG
						// Debug: update stored flag after rx_packet_data
						if (dbg_current_idx < DBG_LS_SIZE) {
							// Check if data was actually stored
							uint8_t rx_pos = dbg_ls_chain_cnt[dbg_current_idx] & 0xFF;  // truncate to 8-bit
							if (app_data_storage[rx_pos] == dbg_ls_data[dbg_current_idx] && dbg_ls_data[dbg_current_idx] != 0) {
								dbg_ls_stored[dbg_current_idx] = 1;
							}
						}
#endif /* PRINT_LEVEL_SHARING_DEBUG */
					}
					// Note: Removed wrong_packet handling - we accept all positions during level sharing
				}
				else
				{
					// CRC failed - just move to next slot (mint_slot++ at end)
					crc_not_ok++;
#if print_extra
					if (chain_count_counter < MAX_NEW)
						app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
					if (app_chain_cnt >= app_chain_len - 1)
					{
						app_state = STATE_MINT_STOP;
					}
				}
			}

			else
			{
				// No address event - just move to next slot
				no_address_event++;
#if print_extra
				if (chain_count_counter < MAX_NEW)
					app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
				if (app_chain_cnt >= app_chain_len - 1)
				{
					app_state = STATE_MINT_STOP;
				}
			}
#if print_extra
			if (seq_counter < MAX_NEW_BIG)
			{
				if (got_wrong_packet)
				{
					mint_tx_status[seq_counter++] = 'U';
				}
				else if (last_rx_ok && last_crc_is_ok)
				{
					mint_tx_status[seq_counter++] = '-';
				}
				else if (!slot_started)
				{
					mint_tx_status[seq_counter++] = 'M';
				}
				else if (!got_address_event)
				{
					mint_tx_status[seq_counter++] = 'A';
				}
				else if (!got_payload_event)
				{
					mint_tx_status[seq_counter++] = 'P';
				}
				else if (!got_end_event)
				{
					mint_tx_status[seq_counter++] = 'E';
				}
				else if (!last_crc_is_ok)
				{
					mint_tx_status[seq_counter++] = 'C';
				}
				else if (!last_rx_ok)
				{
					mint_tx_status[seq_counter++] = 'W'; // wrong address
				}
				else
				{
					mint_tx_status[seq_counter++] = '?';
				}
			}

#endif
		}
		else
		{
			break;
		}
		mint_slot++;
	}
	app_end_time = RTIMER_NOW();
	my_radio_off_completely(); // dbt
	store_levels();
	print_app_states();
}

// ------------------------------------------------------------------------------------------------

static void make_opt_packet(mint_ble_beacon_t *pkt)
{
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
	pkt->radio_len = sizeof(mint_ble_beacon_t) - 1; /* execlude len field */
#else
	pkt->radio_len = sizeof(mint_ble_beacon_t) - 2; /* len + pdu_header */ // length of the rest of the packet

#endif
	pkt->pdu_header = 0x42; // pdu type: 0x02 ADV_NONCONN_IND, rfu 0, rx 0, tx 1 //2;
	//	pkt->adv_address_low = MY_ADV_ADDRESS_LOW;
	//	pkt->adv_address_hi = MY_ADV_ADDRESS_HI;
	//

#if (PACKET_IBEACON_FORMAT)
	//	pkt->ad_flags_length = 2; //2bytes flags
	//	pkt->ad_flags_type = 1; //1=flags
	//	pkt->ad_flags_data = 6; //(non-connectable, undirected advertising, single-mode device)
	//	pkt->ad_length = 0x0b; //26 bytes, the remainder of the packet
	//	pkt->ad_type = 0xff; //manufacturer specific
	//	pkt->company_id = 0x004cU; //Apple ID
	//	pkt->beacon_type = 0x1502;//0x0215U; //proximity ibeacon
	//	pkt->power = 0;//256 - 60; //RSSI = -60 dBm; Measured Power = 256 – 60 = 196 = 0xC4
#endif
	app_pos = APP_CHAIN_CNT_FIELD;

	if (power_array_odd[app_pos])
	{
		// if(power_array[app_pos]) {
		my_radio_set_tx_power(BLE_MAX_POWER);
		fill_data(app_data_odd_storage[app_pos], app_data_len);
	}
	else
	{
		my_radio_set_tx_power(BLE_MIN_POWER);
		fill_data(0, app_data_len);
	}
}

void rx_new_packet_opt_data()
{

	//	if(app_rx_cnt==0){
	//		app_rx_relay_cnt_last = app_relay_cnt_field_backup;
	//	}
	//	if (app_relay_cnt_field_backup > (app_rx_relay_cnt_last)){
	//		app_data_bitmap[app_current_itr++] = app_rx_cnt_main;
	//		app_rx_relay_cnt_last = app_relay_cnt_field_backup;
	//		app_chain_storage[data_counter++] = 0;
	//	}
	app_pos = app_chain_cnt_field_backup;
	if (app_data_odd_storage[app_pos] == 0 && app_data_field_backup != 0)
	{
		app_data_odd_storage[app_pos] = app_data_field_backup;
		power_array_odd[app_pos] = 1;

#if odd_set_initiator
		app_data_storage[odd_chain_app_pos_list[app_pos]] = app_data_field_backup;
		app_time_storage[odd_chain_app_pos_list[app_pos]] = RTIMER_NOW() - app_start_time;
		ntx_recieve[odd_chain_app_pos_list[app_pos]] = ntx_recieve_backup;
		app_rssi[odd_chain_app_pos_list[app_pos]] = app_rssi_field_backup;
		power_array[odd_chain_app_pos_list[app_pos]] = 1;
#else
		app_data_storage[even_chain_app_pos_list[app_pos]] = app_data_field_backup;
		app_time_storage[even_chain_app_pos_list[app_pos]] = RTIMER_NOW() - app_start_time;
		ntx_recieve[even_chain_app_pos_list[app_pos]] = ntx_recieve_backup;
		app_rssi[even_chain_app_pos_list[app_pos]] = app_rssi_field_backup;
		power_array[even_chain_app_pos_list[app_pos]] = 1;
#endif
		app_rx_cnt_main++;
	}
	prev_backup = RTIMER_NOW();
}

void rx_packet_opt_data()
{

	// First Packet received
	//	if(app_rx_cnt==0){
	//		app_rx_relay_cnt_last = app_relay_cnt_field_backup;
	//		if(app_tx_cnt==0)
	//			last_tx_relay_cnt = app_relay_cnt_field_backup-1;
	//	}

	app_pos = app_chain_cnt_field_backup;
	// If the data is not there in the current node and the data recieved by the node in this subslot is non-zero then fdo the operation
	if (app_data_odd_storage[app_pos] == 0 && app_data_field_backup != 0)
	{
		// Save the data received in the data storage array
		app_data_odd_storage[app_pos] = app_data_field_backup;
		power_array_odd[app_pos] = 1;
#if odd_set_initiator
		app_data_storage[odd_chain_app_pos_list[app_pos]] = app_data_field_backup;
		app_time_storage[odd_chain_app_pos_list[app_pos]] = RTIMER_NOW() - app_start_time;
		ntx_recieve[odd_chain_app_pos_list[app_pos]] = ntx_recieve_backup;
		app_rssi[odd_chain_app_pos_list[app_pos]] = app_rssi_field_backup;
		power_array[odd_chain_app_pos_list[app_pos]] = 1;
#else
		app_data_storage[even_chain_app_pos_list[app_pos]] = app_data_field_backup;
		app_time_storage[even_chain_app_pos_list[app_pos]] = RTIMER_NOW() - app_start_time;
		ntx_recieve[even_chain_app_pos_list[app_pos]] = ntx_recieve_backup;
		app_rssi[even_chain_app_pos_list[app_pos]] = app_rssi_field_backup;
		power_array[even_chain_app_pos_list[app_pos]] = 1;
#endif
		app_rx_cnt_main++;
	}

	// If packet received with a new relay count
	//	if (app_relay_cnt_field_backup > app_rx_relay_cnt_last){
	//		//Store the relay count
	//		app_data_bitmap[app_current_itr++] = app_rx_cnt_main;
	//		app_rx_relay_cnt_last = app_relay_cnt_field_backup;
	//		app_chain_storage[data_counter++] = 0;
	//	}
	prev_backup = RTIMER_NOW();
	rssi_backup = (int)((signed char)app_rssi_field_backup) - 45;
	app_rx_cnt++;
}

void try_opt_rx(uint8_t ntx_i)
{
	uint8_t got_payload_event, got_address_event, got_end_event, slot_started, last_crc_is_ok, last_rx_ok;
	got_payload_event = 0, got_address_event = 0, got_end_event = 0, slot_started = 0, last_crc_is_ok = 0, last_rx_ok = 0;
	uint8_t channel = BLE_CHANNEL_37_FREQ;
	rtimer_clock_t rx_target_time, rx_tn, rx_tref, rx_toffset, t_proc;
	uint8_t rx_missed_slot = 0;

	rx_target_time = mint_tt - ADDRESS_EVENT_T_TX_OFFSET - guard_time;
	rx_tn = RTIMER_NOW();
	rx_tref = mint_start_round - FIRST_SLOT_OFFSET;
	rx_toffset = (mint_slot)*MINT_SLOT_LEN + FIRST_SLOT_OFFSET - ADDRESS_EVENT_T_TX_OFFSET - guard_time;
	rx_missed_slot = check_timer_miss(rx_tref, rx_toffset, rx_tn);

	if (!rx_missed_slot)
	{
		// t_proc = RTIMER_NOW();
		schedule_rx_abs(rx_buffer, GET_CHANNEL(mint_round, mint_slot), rx_target_time);
		t_proc = RTIMER_NOW() - rx_tn;
		BUSYWAIT_UNTIL_ABS(NRF_TIMER0->EVENTS_COMPARE[0] != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START);
		slot_started = NRF_TIMER0->EVENTS_COMPARE[0];
		if (slot_started)
		{
			// nrf_gpio_pin_toggle(ROUND_INDICATOR_PIN);
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
			BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_FRAMESTART != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START + ADDRESS_EVENT_T_TX_OFFSET);
			got_address_event = NRF_RADIO->EVENTS_FRAMESTART;
#else
			BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START + ADDRESS_EVENT_T_TX_OFFSET);
			got_address_event = NRF_RADIO->EVENTS_ADDRESS;
#endif
		}
	}

	//}

	if (got_address_event)
	{
		total++;
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
		// no EVENTS_PAYLOAD is emitted
		//  PAYLOAD_AIR_TIME_MIN includes CRC
		BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, get_rx_ts() + PAYLOAD_AIR_TIME_MIN + SLOT_PROCESSING_TIME_PKT_END);

		got_end_event = NRF_RADIO->EVENTS_END;
		last_rx_ok = got_payload_event = got_end_event;
		last_crc_is_ok = USE_HAMMING_CODE || ((got_end_event != 0U) && (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_CRCOk));
		// last_crc_is_ok = 1; //XXX
#else
		BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_PAYLOAD != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN);
		got_payload_event = NRF_RADIO->EVENTS_PAYLOAD;
		last_rx_ok = got_payload_event;
		if (got_payload_event)
		{
			BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN + CRC_AIR_T + SLOT_PROCESSING_TIME_PKT_END);
			got_end_event = NRF_RADIO->EVENTS_END;
			last_crc_is_ok = USE_HAMMING_CODE || ((got_end_event != 0U) && (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_CRCOk));
		}
#endif /* (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit) */
	}
	/* check if it is a valid packet: a. our uuid and b. CRC ok */
	if (last_rx_ok && last_crc_is_ok)
	{
		mint_ble_beacon_t *rx_pkt = (mint_ble_beacon_t *)rx_buffer;
		// #if USE_HAMMING_CODE
		// rx_pkt = (mint_ble_beacon_t *) encode_decode_buffer;
		// last_crc_is_ok = decode_ble_packet(rx_buffer, encode_decode_buffer) == 0;
		// #endif

		/* check if it is our beacon packet */
		//	last_rx_ok = last_crc_is_ok ? (( rx_pkt->adv_address_low == MY_ADV_ADDRESS_LOW ) && ( rx_pkt->adv_address_hi == MY_ADV_ADDRESS_HI )) : 0;
		// last_rx_ok = last_crc_is_ok; //XXX!

		// if(last_rx_ok){
		received++;
		memcpy(&app_packet, &rx_buffer, rx_pkt->radio_len + 2);
		if (APP_CHAIN_CNT_FIELD == app_chain_cnt)
		{
			app_state_backup = app_state;
			app_chain_cnt_field_backup = APP_CHAIN_CNT_FIELD;
			app_data_field_backup = APP_DATA_FIELD[0];
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
			app_rssi_field_backup = app_packet.lqi;
#else
			app_rssi_field_backup = get_radio_rssi();
#endif
			ntx_recieve_backup = ntx_i;

			//	app_relay_cnt_field_backup = APP_RELAY_CNT_FIELD;
			rx_new_packet_opt_data();
		}
		//}
		// else{
		//	error++;
		//}
	}
#if print_extra
	if (seq_counter < MAX_NEW_BIG)
	{
		if (last_rx_ok && last_crc_is_ok)
		{
			mint_tx_status[seq_counter++] = '-';
		}
		else if (!slot_started)
		{
			mint_tx_status[seq_counter++] = 'M';
		}
		else if (!got_address_event)
		{
			mint_tx_status[seq_counter++] = 'A';
		}
		else if (!got_payload_event)
		{
			mint_tx_status[seq_counter++] = 'P';
		}
		else if (!got_end_event)
		{
			mint_tx_status[seq_counter++] = 'E';
		}
		else if (!last_crc_is_ok)
		{
			mint_tx_status[seq_counter++] = 'C';
		}
		else if (!last_rx_ok)
		{
			mint_tx_status[seq_counter++] = 'W'; // wrong address
		}
		else
		{
			mint_tx_status[seq_counter++] = '?';
		}
	}
#endif
}

void initialize_opt_storage()
{
	uint8_t i;

	for (i = 0; i < MAX_NEW; i++)
	{
		app_chain_storage[i] = 0;
		tx_cnts[i] = 0;
		app_data_bitmap[i] = 0;
		app_chain_count[i] = 0;
	}

	for (i = 0; i < CHAIN_LENGTH; i++)
	{
		app_data_storage[i] = 0;
		power_array[i] = 0;
		app_time_storage[i] = 0;
		app_relay_storage[i] = 0;
		app_data_odd_storage[i] = 0;
		power_array_odd[i] = 0;
		ntx_recieve[i] = 0;
		app_rssi[i] = 0;
	}
}

void app_new_opt_start(uint8_t forwarded_data, uint16_t round_)
{
#if print_extra
	if (time_counter < MAX_NEW)
		start_storage[time_counter++] = RTIMER_NOW() - t_start_round;
#endif
	my_level = forwarded_data;
	my_round = round_;
	seq_counter = 0;
	slot_counter = 0;
	chain_count_counter = 0;
	chain_count_no_address_counter = 0;
	rtimer_clock_t target = t_start_round + GLOSSY_DURATION_MAX + MINT_DURATION_MAX + GUARD_TIME;
	// rtimer_clock_t target_2 = RTIMER_NOW() + MINT_DURATION_MAX;
	// time_target_2 = (unsigned long)target_2 * 1e6 / RTIMER_SECOND;
	// time_target = (unsigned long)target * 1e6 / RTIMER_SECOND;

	my_radio_init(&my_id, tx_buffer);
	// app_node_pos = get_testbed_index(my_id,testbed_ids,TESTBED_SIZE) + 1;
	app_node_pos = get_testbed_index(my_id, testbed_ids, TESTBED_SIZE);

	app_num_nodes = NUM_NODES;
	app_n_tx = MINT_N_TX;
	app_num_tx = 1;
	app_tx_max = app_n_tx * app_num_tx;
	//	last_tx_relay_cnt = -1;
	//	app_rx_relay_cnt_last = 0;
	app_rx_cnt_main = 0;
	app_data_len = USER_DATA_LEN;
	app_data = 1;

	initialize_opt_storage();
	app_start_time = RTIMER_NOW();

	if (app_node_pos >= 0 && app_node_pos < app_num_nodes)
	{
		// app_pos = ((app_node_pos-1)/(app_chain_len-2))*app_chain_len + ((app_node_pos-1)%(app_chain_len-2)) + 1;
		app_pos = app_node_pos;
		power_array[app_pos] = 1;
		app_data_storage[app_pos] = testbed_pi_ids[app_node_pos];
		app_time_storage[app_pos] = RTIMER_NOW() - app_start_time;
		app_rssi[app_pos] = 0;

		app_rx_cnt_main++;
	}
	my_initiator_flag = check_initiator_flag(my_level);

	if (my_initiator_flag)
	{
		app_node_pos_optimal = get_odd_list_index(testbed_pi_ids[app_node_pos]);
		power_array_odd[app_node_pos_optimal] = 1;
#if odd_set_initiator
		app_data_odd_storage[app_node_pos_optimal] = odd_chain_list[app_node_pos_optimal];
#else
		app_data_odd_storage[app_node_pos_optimal] = even_chain_list[app_node_pos_optimal];
#endif
	}
#if odd_set_initiator
	app_chain_len = odd_chain;
#else
	app_chain_len = even_chain;

#endif

	app_tx_cnt = 0;
	app_rx_cnt = 0;
	wrong_address = 0;
	address_event = 0;
	received = 0;
	crc_not_ok = 0;
	app_chain_cnt_no_address = 0;
	no_address_event = 0;
	guard_time = GUARD_TIME;
	app_current_itr = 0;
	app_chain_cnt = 0;
	data_counter = 0;
	radio_on_time = 0;
	//	APP_RELAY_CNT_FIELD = 0;
	APP_CHAIN_CNT_FIELD = 0;

	app_packet_len = sizeof(mint_ble_beacon_t);

	// if(IS_MINT_INITIATOR())
	if (my_initiator_flag)
	{
		app_state = STATE_TX_NOT_END;
	}
	else
	{
		app_state = STATE_RX_NOT_END;
	}
	mint_start_round = t_start_round + GLOSSY_DURATION_MAX + FIRST_SLOT_OFFSET;

	mint_round = round;
	mint_slot = 0;
	slot_terminator = 0;
	ntx_itr = 1;

#if print_extra
	mint_tx_status[seq_counter] = 'S';
	seq_counter++;
	round_storage[round_counter++] = mint_round;
#endif
	while (RTIMER_CLOCK_LT(RTIMER_NOW(), target))
	{
		slot_terminator++;
		if (slot_terminator == app_chain_len + 1)
		{
#if print_extra
			if (seq_counter < MAX_NEW_BIG)
				mint_tx_status[seq_counter++] = 'X';
#endif
			slot_terminator = 1;
			one_subslot_packet_received_flag = 0;
			app_chain_cnt = 0;
			app_chain_len = CHAIN_LENGTH;
			ntx_itr++;
		}

		mint_tt = mint_start_round + (mint_slot)*MINT_SLOT_LEN;
		if (ntx_itr == 1)
		{
			if (app_state == STATE_TX_NOT_END)
			{
				if (power_array_odd[APP_CHAIN_CNT_FIELD] == 1)
				{
					guard_time = GUARD_TIME_SHORT;
					make_opt_packet(&app_packet);
					uint8_t *tx_msg = (uint8_t *)&app_packet;
					schedule_tx_abs(tx_msg, GET_CHANNEL(mint_round, mint_slot), mint_tt - ADDRESS_EVENT_T_TX_OFFSET + ARTIFICIAL_TX_OFFSET);

					BUSYWAIT_UNTIL_ABS(NRF_TIMER0->EVENTS_COMPARE[0] != 0U, mint_tt - ADDRESS_EVENT_T_TX_OFFSET + ARTIFICIAL_TX_OFFSET);
					if (!NRF_TIMER0->EVENTS_COMPARE[0])
					{
#if print_extra
						if (seq_counter < MAX_NEW_BIG)
							mint_tx_status[seq_counter++] = 'T';
#endif
					}
					else
					{
						BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, mint_tt + ARTIFICIAL_TX_OFFSET + MINT_PACKET_AIR_TIME_MIN);
						if (!NRF_RADIO->EVENTS_END)
						{
#if print_extra
							if (seq_counter < MAX_NEW_BIG)
								mint_tx_status[seq_counter++] = 'R';
#endif
						}
						else
						{
#if print_extra
							if (seq_counter < MAX_NEW_BIG)
								mint_tx_status[seq_counter++] = 'B';
#endif
						}
					}
					if (app_chain_cnt == app_chain_len - 1)
					{
						app_state = STATE_RX_NOT_END;
					}
					// every other transmission needs to be immediately followed by another transmission
					else
					{
						app_state = STATE_TX_NOT_END;
					}

					//	last_tx_relay_cnt = APP_RELAY_CNT_FIELD;

					if (app_state == STATE_TX_NOT_END)
					{
#if print_extra
						if (chain_count_counter < MAX_NEW)
							app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
						app_chain_cnt++;
						APP_CHAIN_CNT_FIELD = app_chain_cnt;
					}
					else
					{
#if print_extra
						if (chain_count_counter < MAX_NEW)
							app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
						app_chain_cnt = 0;
						tx_cnts[app_tx_cnt] = app_tx_cnt + 1;
						app_tx_cnt++;
						if ((app_tx_cnt == app_tx_max) /*&& !IS_MINT_INITIATOR()*/)
						{
							app_state = STATE_MINT_STOP;
						}
					}
				}
				else
				{
					try_opt_rx(ntx_itr);
					if (app_chain_cnt == app_chain_len - 1)
					{
#if print_extra
						if (chain_count_counter < MAX_NEW)
							app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif

						app_chain_cnt = 0;
						app_state = STATE_RX_NOT_END;
						app_tx_cnt++;
						if ((app_tx_cnt == app_tx_max) /*&& !IS_MINT_INITIATOR()*/)
						{
							app_state = STATE_MINT_STOP;
						}
					}
					else
					{
#if print_extra
						if (chain_count_counter < MAX_NEW)
							app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
						app_state = STATE_TX_NOT_END;
						app_chain_cnt++;
						APP_CHAIN_CNT_FIELD = app_chain_cnt;
					}
				}
			}
			else if (app_state == STATE_RX_NOT_END)
			{
				guard_time = GUARD_TIME;
				uint8_t got_payload_event, got_address_event, got_end_event, slot_started, last_crc_is_ok, last_rx_ok, got_wrong_packet;
				got_payload_event = 0, got_address_event = 0, got_end_event = 0, slot_started = 0, last_crc_is_ok = 0, last_rx_ok = 0, got_wrong_packet = 0;
				uint8_t channel = BLE_CHANNEL_37_FREQ;
				rtimer_clock_t rx_target_time, rx_tn, rx_tref, rx_toffset, t_proc;
				uint8_t rx_missed_slot = 0;
				rx_target_time = mint_tt - ADDRESS_EVENT_T_TX_OFFSET - guard_time;
				rx_tn = RTIMER_NOW();
				rx_tref = mint_start_round - FIRST_SLOT_OFFSET;
				rx_toffset = (mint_slot)*MINT_SLOT_LEN + FIRST_SLOT_OFFSET - ADDRESS_EVENT_T_TX_OFFSET - guard_time;
				rx_missed_slot = check_timer_miss(rx_tref, rx_toffset, rx_tn);
				if (!rx_missed_slot)
				{
					schedule_rx_abs(rx_buffer, GET_CHANNEL(mint_round, mint_slot), rx_target_time);
					t_proc = RTIMER_NOW() - rx_tn;
					BUSYWAIT_UNTIL_ABS(NRF_TIMER0->EVENTS_COMPARE[0] != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START);
					slot_started = NRF_TIMER0->EVENTS_COMPARE[0];
					if (slot_started)
					{
						// nrf_gpio_pin_toggle(ROUND_INDICATOR_PIN);
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
						BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_FRAMESTART != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START + ADDRESS_EVENT_T_TX_OFFSET);
						got_address_event = NRF_RADIO->EVENTS_FRAMESTART;
#else
						BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START + ADDRESS_EVENT_T_TX_OFFSET);
						got_address_event = NRF_RADIO->EVENTS_ADDRESS;
#endif
					}
				}

				if (got_address_event)
				{
					address_event++;
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
					// no EVENTS_PAYLOAD is emitted
					//  PAYLOAD_AIR_TIME_MIN includes CRC
					BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN + SLOT_PROCESSING_TIME_PKT_END);

					got_end_event = NRF_RADIO->EVENTS_END;
					last_rx_ok = got_payload_event = got_end_event;
					last_crc_is_ok = USE_HAMMING_CODE || ((got_end_event != 0U) && (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_CRCOk));
					// last_crc_is_ok = 1; //XXX
#else
					BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_PAYLOAD != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN);
					got_payload_event = NRF_RADIO->EVENTS_PAYLOAD;
					last_rx_ok = got_payload_event;
					if (got_payload_event)
					{
						BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN + CRC_AIR_T + SLOT_PROCESSING_TIME_PKT_END);
						got_end_event = NRF_RADIO->EVENTS_END;
						last_crc_is_ok = USE_HAMMING_CODE || ((got_end_event != 0U) && (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_CRCOk));
					}
#endif /* (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit) */

					/* check if it is a valid packet: a. our uuid and b. CRC ok */
					if (last_rx_ok && last_crc_is_ok)
					{
						mint_ble_beacon_t *rx_pkt = (mint_ble_beacon_t *)rx_buffer;

						/* check if it is our beacon packet */
						//	last_rx_ok = last_crc_is_ok ? (( rx_pkt->adv_address_low == MY_ADV_ADDRESS_LOW ) && ( rx_pkt->adv_address_hi == MY_ADV_ADDRESS_HI )) : 0;
						// last_rx_ok = last_crc_is_ok; //XXX!

						//	if(last_rx_ok){

						memcpy(&app_packet, &rx_buffer, rx_pkt->radio_len + 2);
						if (APP_CHAIN_CNT_FIELD == app_chain_cnt)
						{
							received++;

							if (APP_CHAIN_CNT_FIELD == app_chain_len - 1)
							{
								app_state = STATE_TX_NOT_END;
							}

							app_state_backup = app_state;
							app_chain_cnt_field_backup = APP_CHAIN_CNT_FIELD;
							app_data_field_backup = APP_DATA_FIELD[0];
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
							app_rssi_field_backup = app_packet.lqi;
#else
							app_rssi_field_backup = get_radio_rssi();
#endif
							// app_relay_cnt_field_backup = APP_RELAY_CNT_FIELD;
							one_subslot_packet_received_flag = 1;
							ntx_recieve_backup = ntx_itr;

							if (app_state == STATE_RX_NOT_END)
							{
#if print_extra
								if (chain_count_counter < MAX_NEW)
									app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
								app_chain_cnt++;
							}
							else
							{
#if print_extra
								if (chain_count_counter < MAX_NEW)
								{
									app_chain_count[chain_count_counter++] = app_chain_cnt;
								}
								if (chain_count_no_address_counter < MAX_NEW_BIG && app_chain_cnt_no_address != 0)
								{
									app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
								}
#endif
								app_chain_cnt = 0;
								app_chain_cnt_no_address = 0;
								APP_CHAIN_CNT_FIELD = app_chain_cnt;
								// APP_RELAY_CNT_FIELD++;
								if (app_tx_cnt == app_tx_max)
								{
									// no more Tx to perform: stop Glossy
									app_state = STATE_MINT_STOP;
								}
							}

							rx_packet_opt_data();
						}
						else
						{
							wrong_packet++;
							got_wrong_packet = 1;
							if (app_chain_cnt == app_chain_len - 1)
							{
#if print_extra
								if (chain_count_counter < MAX_NEW)
									app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
								app_state = STATE_TX_NOT_END;
							}
							if (app_state == STATE_RX_NOT_END)
							{
#if print_extra
								if (chain_count_counter < MAX_NEW)
									app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
								app_chain_cnt++;
							}
							else
							{
#if print_extra
								if (chain_count_counter < MAX_NEW)
								{
									app_chain_count[chain_count_counter++] = app_chain_cnt;
								}
								if (chain_count_no_address_counter < MAX_NEW_BIG && app_chain_cnt_no_address != 0)
								{
									app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
								}
#endif
								app_chain_cnt = 0;
								app_chain_cnt_no_address = 0;
								APP_CHAIN_CNT_FIELD = app_chain_cnt;
								//	APP_RELAY_CNT_FIELD=app_relay_cnt_field_backup+1;
								if (app_tx_cnt == app_tx_max)
								{
									// no more Tx to perform: stop Glossy
									app_state = STATE_MINT_STOP;
								}
							}
						}
						//					}

						//						else{
						//							wrong_address++;
						//							if(app_chain_cnt == (app_chain_len-1) && one_subslot_packet_received_flag==1)
						//							{
						// #if print_extra
						//								if(chain_count_counter < MAX_NEW)
						//								{
						//									app_chain_count[chain_count_counter++] = app_chain_cnt;
						//								}
						//								if(chain_count_no_address_counter < MAX_NEW_BIG && app_chain_cnt_no_address!=0)
						//								{
						//									app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
						//								}
						//
						// #endif
						//								app_state = STATE_TX_NOT_END;
						//								app_chain_cnt = 0;
						//								app_chain_cnt_no_address=0;
						//								APP_CHAIN_CNT_FIELD = app_chain_cnt;
						//								APP_RELAY_CNT_FIELD=app_relay_cnt_field_backup+1;
						//								if (app_tx_cnt == app_tx_max) {
						//									app_state = STATE_MINT_STOP;
						//								}
						//							}
						//							if(app_state == STATE_RX_NOT_END){
						// #if print_extra
						//								if(chain_count_counter < MAX_NEW)
						//									app_chain_count[chain_count_counter++] = app_chain_cnt;
						// #endif
						//								app_chain_cnt++;
						//							}
						//						}
					}
					else
					{
						crc_not_ok++;
						if (app_chain_cnt == (app_chain_len - 1) && one_subslot_packet_received_flag == 1)
						{
#if print_extra
							if (chain_count_counter < MAX_NEW)
							{
								app_chain_count[chain_count_counter++] = app_chain_cnt;
							}
							if (chain_count_no_address_counter < MAX_NEW_BIG && app_chain_cnt_no_address != 0)
							{
								app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
							}

#endif
							app_state = STATE_TX_NOT_END;
							app_chain_cnt = 0;
							app_chain_cnt_no_address = 0;
							APP_CHAIN_CNT_FIELD = app_chain_cnt;
							// APP_RELAY_CNT_FIELD=app_relay_cnt_field_backup+1;
							if (app_tx_cnt == app_tx_max)
							{
								app_state = STATE_MINT_STOP;
							}
						}
						if (app_state == STATE_RX_NOT_END)
						{
#if print_extra
							if (chain_count_counter < MAX_NEW)
								app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
							app_chain_cnt++;
						}
					}
				}

				else
				{
					no_address_event++;
					if (app_chain_cnt == (app_chain_len - 1) && one_subslot_packet_received_flag == 1)
					{

#if print_extra
						if (chain_count_counter < MAX_NEW)
						{
							app_chain_count[chain_count_counter++] = app_chain_cnt;
						}
						if (chain_count_no_address_counter < MAX_NEW_BIG && app_chain_cnt_no_address != 0)
						{
							app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
						}
#endif
						app_state = STATE_TX_NOT_END;
						app_chain_cnt = 0;
						app_chain_cnt_no_address = 0;
						APP_CHAIN_CNT_FIELD = app_chain_cnt;
						//	APP_RELAY_CNT_FIELD=app_relay_cnt_field_backup+1;
						if (app_tx_cnt == app_tx_max)
						{
							app_state = STATE_MINT_STOP;
						}
					}
					else if (app_chain_cnt_no_address >= 10 * (CHAIN_LENGTH))
					{
						app_state = STATE_MINT_STOP;
#if print_extra
						if (chain_count_counter < MAX_NEW)
						{
							app_chain_count[chain_count_counter++] = app_chain_cnt;
						}
						if (chain_count_no_address_counter < MAX_NEW_BIG)
						{
							app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
						}
#endif
					}
					else if (app_state == STATE_RX_NOT_END)
					{
#if print_extra
						if (chain_count_counter < MAX_NEW)
							app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
						app_chain_cnt++;
						app_chain_cnt_no_address++;
					}
					else
					{
					}
				}
#if print_extra
				if (seq_counter < MAX_NEW_BIG)
				{
					if (got_wrong_packet)
					{
						mint_tx_status[seq_counter++] = 'U';
					}
					else if (last_rx_ok && last_crc_is_ok)
					{
						mint_tx_status[seq_counter++] = '-';
					}
					else if (!slot_started)
					{
						mint_tx_status[seq_counter++] = 'M';
					}
					else if (!got_address_event)
					{
						mint_tx_status[seq_counter++] = 'A';
					}
					else if (!got_payload_event)
					{
						mint_tx_status[seq_counter++] = 'P';
					}
					else if (!got_end_event)
					{
						mint_tx_status[seq_counter++] = 'E';
					}
					else if (!last_crc_is_ok)
					{
						mint_tx_status[seq_counter++] = 'C';
					}
					else if (!last_rx_ok)
					{
						mint_tx_status[seq_counter++] = 'W'; // wrong address
					}
					else
					{
						mint_tx_status[seq_counter++] = '?';
					}
				}

#endif
			}
			else
			{
				break;
			}
		}
		else
		{
			if (app_state == STATE_TX_NOT_END)
			{
				if (power_array[APP_CHAIN_CNT_FIELD] == 1)
				{
					guard_time = GUARD_TIME_SHORT;
					make_packet(&app_packet);
					uint8_t *tx_msg = (uint8_t *)&app_packet;
					schedule_tx_abs(tx_msg, GET_CHANNEL(mint_round, mint_slot), mint_tt - ADDRESS_EVENT_T_TX_OFFSET + ARTIFICIAL_TX_OFFSET);

					BUSYWAIT_UNTIL_ABS(NRF_TIMER0->EVENTS_COMPARE[0] != 0U, mint_tt - ADDRESS_EVENT_T_TX_OFFSET + ARTIFICIAL_TX_OFFSET);
					if (!NRF_TIMER0->EVENTS_COMPARE[0])
					{
#if print_extra
						if (seq_counter < MAX_NEW_BIG)
							mint_tx_status[seq_counter++] = 'T';
#endif
					}
					else
					{
						BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, mint_tt + ARTIFICIAL_TX_OFFSET + MINT_PACKET_AIR_TIME_MIN);
						if (!NRF_RADIO->EVENTS_END)
						{
#if print_extra
							if (seq_counter < MAX_NEW_BIG)
								mint_tx_status[seq_counter++] = 'R';
#endif
						}
						else
						{
#if print_extra
							if (seq_counter < MAX_NEW_BIG)
								mint_tx_status[seq_counter++] = 'B';
#endif
						}
					}
					if (app_chain_cnt == app_chain_len - 1)
					{
						app_state = STATE_RX_NOT_END;
					}
					// every other transmission needs to be immediately followed by another transmission
					else
					{
						app_state = STATE_TX_NOT_END;
					}

					//	last_tx_relay_cnt = APP_RELAY_CNT_FIELD;

					if (app_state == STATE_TX_NOT_END)
					{
#if print_extra
						if (chain_count_counter < MAX_NEW)
							app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
						app_chain_cnt++;
						APP_CHAIN_CNT_FIELD = app_chain_cnt;
					}
					else
					{
#if print_extra
						if (chain_count_counter < MAX_NEW)
							app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
						app_chain_cnt = 0;
						tx_cnts[app_tx_cnt] = app_tx_cnt + 1;
						app_tx_cnt++;
						if ((app_tx_cnt == app_tx_max) /*&& !IS_MINT_INITIATOR()*/)
						{
							app_state = STATE_MINT_STOP;
						}
					}
				}
				else
				{
					try_rx(ntx_itr);
					if (app_chain_cnt == app_chain_len - 1)
					{
#if print_extra
						if (chain_count_counter < MAX_NEW)
							app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif

						app_chain_cnt = 0;
						app_state = STATE_RX_NOT_END;
						app_tx_cnt++;
						if ((app_tx_cnt == app_tx_max) /*&& !IS_MINT_INITIATOR()*/)
						{
							app_state = STATE_MINT_STOP;
						}
					}
					else
					{
#if print_extra
						if (chain_count_counter < MAX_NEW)
							app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
						app_state = STATE_TX_NOT_END;
						app_chain_cnt++;
						APP_CHAIN_CNT_FIELD = app_chain_cnt;
					}
				}
			}
			else if (app_state == STATE_RX_NOT_END)
			{
				guard_time = GUARD_TIME;
				uint8_t got_payload_event, got_address_event, got_end_event, slot_started, last_crc_is_ok, last_rx_ok, got_wrong_packet;
				got_payload_event = 0, got_address_event = 0, got_end_event = 0, slot_started = 0, last_crc_is_ok = 0, last_rx_ok = 0, got_wrong_packet = 0;
				uint8_t channel = BLE_CHANNEL_37_FREQ;
				rtimer_clock_t rx_target_time, rx_tn, rx_tref, rx_toffset, t_proc;
				uint8_t rx_missed_slot = 0;
				rx_target_time = mint_tt - ADDRESS_EVENT_T_TX_OFFSET - guard_time;
				rx_tn = RTIMER_NOW();
				rx_tref = mint_start_round - FIRST_SLOT_OFFSET;
				rx_toffset = (mint_slot)*MINT_SLOT_LEN + FIRST_SLOT_OFFSET - ADDRESS_EVENT_T_TX_OFFSET - guard_time;
				rx_missed_slot = check_timer_miss(rx_tref, rx_toffset, rx_tn);
				if (!rx_missed_slot)
				{
					schedule_rx_abs(rx_buffer, GET_CHANNEL(mint_round, mint_slot), rx_target_time);
					t_proc = RTIMER_NOW() - rx_tn;
					BUSYWAIT_UNTIL_ABS(NRF_TIMER0->EVENTS_COMPARE[0] != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START);
					slot_started = NRF_TIMER0->EVENTS_COMPARE[0];
					if (slot_started)
					{
						// nrf_gpio_pin_toggle(ROUND_INDICATOR_PIN);
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
						BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_FRAMESTART != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START + ADDRESS_EVENT_T_TX_OFFSET);
						got_address_event = NRF_RADIO->EVENTS_FRAMESTART;
#else
						BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_ADDRESS != 0U, rx_target_time + 2 * guard_time + SLOT_PROCESSING_TIME_PKT_START + ADDRESS_EVENT_T_TX_OFFSET);
						got_address_event = NRF_RADIO->EVENTS_ADDRESS;
#endif
					}
				}

				if (got_address_event)
				{
					address_event++;
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
					// no EVENTS_PAYLOAD is emitted
					//  PAYLOAD_AIR_TIME_MIN includes CRC
					BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN + SLOT_PROCESSING_TIME_PKT_END);

					got_end_event = NRF_RADIO->EVENTS_END;
					last_rx_ok = got_payload_event = got_end_event;
					last_crc_is_ok = USE_HAMMING_CODE || ((got_end_event != 0U) && (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_CRCOk));
					// last_crc_is_ok = 1; //XXX
#else
					BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_PAYLOAD != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN);
					got_payload_event = NRF_RADIO->EVENTS_PAYLOAD;
					last_rx_ok = got_payload_event;
					if (got_payload_event)
					{
						BUSYWAIT_UNTIL_ABS(NRF_RADIO->EVENTS_END != 0U, get_rx_ts() + MINT_PAYLOAD_AIR_TIME_MIN + CRC_AIR_T + SLOT_PROCESSING_TIME_PKT_END);
						got_end_event = NRF_RADIO->EVENTS_END;
						last_crc_is_ok = USE_HAMMING_CODE || ((got_end_event != 0U) && (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_CRCOk));
					}
#endif /* (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit) */

					/* check if it is a valid packet: a. our uuid and b. CRC ok */
					if (last_rx_ok && last_crc_is_ok)
					{
						mint_ble_beacon_t *rx_pkt = (mint_ble_beacon_t *)rx_buffer;

						/* check if it is our beacon packet */
						//	last_rx_ok = last_crc_is_ok ? (( rx_pkt->adv_address_low == MY_ADV_ADDRESS_LOW ) && ( rx_pkt->adv_address_hi == MY_ADV_ADDRESS_HI )) : 0;
						// last_rx_ok = last_crc_is_ok; //XXX!

						//						if(last_rx_ok){

						memcpy(&app_packet, &rx_buffer, rx_pkt->radio_len + 2);
						if (APP_CHAIN_CNT_FIELD == app_chain_cnt)
						{
							received++;

							if (APP_CHAIN_CNT_FIELD == app_chain_len - 1)
							{
								app_state = STATE_TX_NOT_END;
							}

							app_state_backup = app_state;
							app_chain_cnt_field_backup = APP_CHAIN_CNT_FIELD;
							app_data_field_backup = APP_DATA_FIELD[0];
#if (RADIO_MODE_CONF == RADIO_MODE_MODE_Ieee802154_250Kbit)
							app_rssi_field_backup = app_packet.lqi;
#else
							app_rssi_field_backup = get_radio_rssi();
#endif
							//	app_relay_cnt_field_backup = APP_RELAY_CNT_FIELD;
							one_subslot_packet_received_flag = 1;
							ntx_recieve_backup = ntx_itr;

							if (app_state == STATE_RX_NOT_END)
							{
#if print_extra
								if (chain_count_counter < MAX_NEW)
									app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
								app_chain_cnt++;
							}
							else
							{
#if print_extra
								if (chain_count_counter < MAX_NEW)
								{
									app_chain_count[chain_count_counter++] = app_chain_cnt;
								}
								if (chain_count_no_address_counter < MAX_NEW_BIG && app_chain_cnt_no_address != 0)
								{
									app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
								}
#endif
								app_chain_cnt = 0;
								app_chain_cnt_no_address = 0;
								APP_CHAIN_CNT_FIELD = app_chain_cnt;
								//	APP_RELAY_CNT_FIELD++;
								if (app_tx_cnt == app_tx_max)
								{
									// no more Tx to perform: stop Glossy
									app_state = STATE_MINT_STOP;
								}
							}

							rx_packet_data();
						}
						else
						{
							wrong_packet++;
							got_wrong_packet = 1;
							if (app_chain_cnt == app_chain_len - 1)
							{
#if print_extra
								if (chain_count_counter < MAX_NEW)
									app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
								app_state = STATE_TX_NOT_END;
							}
							if (app_state == STATE_RX_NOT_END)
							{
#if print_extra
								if (chain_count_counter < MAX_NEW)
									app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
								app_chain_cnt++;
							}
							else
							{
#if print_extra
								if (chain_count_counter < MAX_NEW)
								{
									app_chain_count[chain_count_counter++] = app_chain_cnt;
								}
								if (chain_count_no_address_counter < MAX_NEW_BIG && app_chain_cnt_no_address != 0)
								{
									app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
								}
#endif
								app_chain_cnt = 0;
								app_chain_cnt_no_address = 0;
								APP_CHAIN_CNT_FIELD = app_chain_cnt;
								// APP_RELAY_CNT_FIELD=app_relay_cnt_field_backup+1;
								if (app_tx_cnt == app_tx_max)
								{
									// no more Tx to perform: stop Glossy
									app_state = STATE_MINT_STOP;
								}
							}
						}
						//						}

						//						else{
						//							wrong_address++;
						//							if(app_chain_cnt == (app_chain_len-1) && one_subslot_packet_received_flag==1)
						//							{
						//	#if print_extra
						//								if(chain_count_counter < MAX_NEW)
						//								{
						//									app_chain_count[chain_count_counter++] = app_chain_cnt;
						//								}
						//								if(chain_count_no_address_counter < MAX_NEW_BIG && app_chain_cnt_no_address!=0)
						//								{
						//									app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
						//								}
						//
						//	#endif
						//								app_state = STATE_TX_NOT_END;
						//								app_chain_cnt = 0;
						//								app_chain_cnt_no_address=0;
						//								APP_CHAIN_CNT_FIELD = app_chain_cnt;
						//								APP_RELAY_CNT_FIELD=app_relay_cnt_field_backup+1;
						//								if (app_tx_cnt == app_tx_max) {
						//									app_state = STATE_MINT_STOP;
						//								}
						//							}
						//							if(app_state == STATE_RX_NOT_END){
						//	#if print_extra
						//								if(chain_count_counter < MAX_NEW)
						//									app_chain_count[chain_count_counter++] = app_chain_cnt;
						//	#endif
						//								app_chain_cnt++;
						//							}
						//						}
					}
					else
					{
						crc_not_ok++;
						if (app_chain_cnt == (app_chain_len - 1) && one_subslot_packet_received_flag == 1)
						{
#if print_extra
							if (chain_count_counter < MAX_NEW)
							{
								app_chain_count[chain_count_counter++] = app_chain_cnt;
							}
							if (chain_count_no_address_counter < MAX_NEW_BIG && app_chain_cnt_no_address != 0)
							{
								app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
							}

#endif
							app_state = STATE_TX_NOT_END;
							app_chain_cnt = 0;
							app_chain_cnt_no_address = 0;
							APP_CHAIN_CNT_FIELD = app_chain_cnt;
							// APP_RELAY_CNT_FIELD=app_relay_cnt_field_backup+1;
							if (app_tx_cnt == app_tx_max)
							{
								app_state = STATE_MINT_STOP;
							}
						}
						if (app_state == STATE_RX_NOT_END)
						{
#if print_extra
							if (chain_count_counter < MAX_NEW)
								app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
							app_chain_cnt++;
						}
					}
				}

				else
				{
					no_address_event++;
					if (app_chain_cnt == (app_chain_len - 1) && one_subslot_packet_received_flag == 1)
					{

#if print_extra
						if (chain_count_counter < MAX_NEW)
						{
							app_chain_count[chain_count_counter++] = app_chain_cnt;
						}
						if (chain_count_no_address_counter < MAX_NEW_BIG && app_chain_cnt_no_address != 0)
						{
							app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
						}
#endif
						app_state = STATE_TX_NOT_END;
						app_chain_cnt = 0;
						app_chain_cnt_no_address = 0;
						APP_CHAIN_CNT_FIELD = app_chain_cnt;
						// APP_RELAY_CNT_FIELD=app_relay_cnt_field_backup+1;
						if (app_tx_cnt == app_tx_max)
						{
							app_state = STATE_MINT_STOP;
						}
					}
					else if (app_chain_cnt_no_address >= 10 * (CHAIN_LENGTH))
					{
						app_state = STATE_MINT_STOP;
#if print_extra
						if (chain_count_counter < MAX_NEW)
						{
							app_chain_count[chain_count_counter++] = app_chain_cnt;
						}
						if (chain_count_no_address_counter < MAX_NEW_BIG)
						{
							app_chain_count_no_address[chain_count_no_address_counter++] = app_chain_cnt_no_address;
						}
#endif
					}
					else if (app_state == STATE_RX_NOT_END)
					{
#if print_extra
						if (chain_count_counter < MAX_NEW)
							app_chain_count[chain_count_counter++] = app_chain_cnt;
#endif
						app_chain_cnt++;
						app_chain_cnt_no_address++;
					}
					else
					{
					}
				}
#if print_extra
				if (seq_counter < MAX_NEW_BIG)
				{
					if (got_wrong_packet)
					{
						mint_tx_status[seq_counter++] = 'U';
					}
					else if (last_rx_ok && last_crc_is_ok)
					{
						mint_tx_status[seq_counter++] = '-';
					}
					else if (!slot_started)
					{
						mint_tx_status[seq_counter++] = 'M';
					}
					else if (!got_address_event)
					{
						mint_tx_status[seq_counter++] = 'A';
					}
					else if (!got_payload_event)
					{
						mint_tx_status[seq_counter++] = 'P';
					}
					else if (!got_end_event)
					{
						mint_tx_status[seq_counter++] = 'E';
					}
					else if (!last_crc_is_ok)
					{
						mint_tx_status[seq_counter++] = 'C';
					}
					else if (!last_rx_ok)
					{
						mint_tx_status[seq_counter++] = 'W'; // wrong address
					}
					else
					{
						mint_tx_status[seq_counter++] = '?';
					}
				}

#endif
			}
			else
			{
				break;
			}
		}
		mint_slot++;
	}
	app_end_time = RTIMER_NOW();
	my_radio_off_completely(); // dbt
	print_app_states();
}
