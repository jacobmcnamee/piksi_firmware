/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ch.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/bit_sync.h>

#include "board/nap/track_channel.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "track.h"
#include "simulator.h"
#include "peripherals/random.h"
#include "settings.h"
#include "signal.h"



/** \defgroup tracking Tracking
 * Track satellites via interrupt driven updates to SwiftNAP tracking channels.
 * Initialize SwiftNAP tracking channels. Run loop filters and update
 * channels' code / carrier frequencies each integration period. Update
 * tracking measurements each integration period.
 * \{ */

#define COMPILER_BARRIER() asm volatile ("" : : : "memory")

#define GPS_WEEK_LENGTH_ms (1000 * WEEK_SECS)
#define CHANNEL_SHUTDOWN_TIME_ms 100

#define NAV_BIT_FIFO_INDEX_MASK ((NAV_BIT_FIFO_SIZE) - 1)
#define NAV_BIT_FIFO_INDEX_DIFF(write_index, read_index) \
          ((nav_bit_fifo_index_t)((write_index) - (read_index)))
#define NAV_BIT_FIFO_LENGTH(p_fifo) \
          (NAV_BIT_FIFO_INDEX_DIFF((p_fifo)->write_index, (p_fifo)->read_index))


static u16 iq_output_mask = 0;

#define NAV_BIT_FIFO_SIZE 32 /**< Size of nav bit FIFO. Must be a power of 2 */

typedef struct {
  s8 soft_bit;
} nav_bit_fifo_element_t;

typedef u8 nav_bit_fifo_index_t;

typedef struct {
  nav_bit_fifo_index_t read_index;
  nav_bit_fifo_index_t write_index;
  nav_bit_fifo_element_t elements[NAV_BIT_FIFO_SIZE];
} nav_bit_fifo_t;

typedef struct {
  s32 TOW_ms;
  s8 bit_polarity;
  nav_bit_fifo_index_t read_index;
  bool valid;
} nav_time_sync_t;

#define NUM_TRACKER_CHANNELS  12

typedef enum {
  STATE_DISABLED,
  STATE_ENABLED,
  STATE_DISABLE_REQUESTED
} state_t;

typedef enum {
  EVENT_ENABLE,
  EVENT_DISABLE_REQUEST,
  EVENT_DISABLE
} event_t;

typedef struct {
  /** FIFO for navigation message bits. */
  nav_bit_fifo_t nav_bit_fifo;
  /** Used to sync time decoded from navigation message
   * back to tracking channel. */
  nav_time_sync_t nav_time_sync;
  /** Time since last nav bit was appended to the nav bit FIFO. */
  u32 nav_bit_TOW_offset_ms;
  /** Bit sync state. */
  bit_sync_t bit_sync;
  /** Polarity of nav message bits. */
  s8 bit_polarity;
  /** Increments when tracking new signal. */
  u16 lock_counter;
  /** Time at which the channel was disabled. */
  systime_t disable_time;
  /** Set if this channel should output I/Q samples on SBP. */
  bool output_iq;
  /** Elevation angle, degrees */
  s8 elevation;
} tracker_internal_data_t;

/** Top-level generic tracker channel. */
typedef struct {
  /** State of this channel. */
  state_t state;
  /** Info associated with this channel. */
  tracker_channel_info_t info;
  /** Mutex used to protect access to common_data. */
  Mutex common_data_mutex;
  /** Data common to all tracker implementations. */
  tracker_common_data_t common_data;
  /** Internal data used by the core module for all tracker implementations. */
  tracker_internal_data_t internal_data;
  /** Mutex used to handle NAP shutdown delay. */
  Mutex nap_mutex;
  /** Associated tracker instance. */
  tracker_t *tracker;
} tracker_channel_t;

static tracker_interface_list_element_t *tracker_interface_list = 0;
static tracker_channel_t tracker_channels[NUM_TRACKER_CHANNELS];

static WORKING_AREA_CCM(wa_track_thread, 3000);

static msg_t track_thread(void *arg);


static tracker_channel_t * tracker_channel_get(tracker_channel_id_t
                                               tracker_channel_id);
static const tracker_interface_t * tracker_interface_get(gnss_signal_t sid);
static bool available_tracker_channel_get(tracker_channel_t **tracker_channel);
static bool available_tracker_get(const tracker_interface_t *interface,
                                  tracker_t **tracker);
static state_t tracker_channel_state_get(const tracker_channel_t *
                                         tracker_channel);
static bool tracker_active(const tracker_t *tracker);
static void interface_function(tracker_channel_t *tracker_channel,
                               tracker_interface_function_t func);
static void event(tracker_channel_t *d, event_t event);

static const tracker_interface_t tracker_interface_default = {
  .code =         CODE_INVALID,
  .init =         0,
  .disable =      0,
  .process =      0,
  .trackers =     0,
  .num_trackers = 0
};

/* signal lock counter
 * A map of signal to an initially random number that increments each time that
 * signal begins being tracked.
 */
static u16 tracking_lock_counters[PLATFORM_SIGNAL_COUNT];

static MUTEX_DECL(nav_time_sync_mutex);

static void nav_bit_fifo_init(nav_bit_fifo_t *fifo);
static bool nav_bit_fifo_write(nav_bit_fifo_t *fifo,
                               const nav_bit_fifo_element_t *element);
static bool nav_bit_fifo_read(nav_bit_fifo_t *fifo,
                              nav_bit_fifo_element_t *element);
static void nav_time_sync_init(nav_time_sync_t *sync);
static bool nav_time_sync_set(nav_time_sync_t *sync, s32 TOW_ms,
                              s8 bit_polarity, nav_bit_fifo_index_t read_index);
static bool nav_time_sync_get(nav_time_sync_t *sync, s32 *TOW_ms,
                              s8 *bit_polarity, nav_bit_fifo_index_t *read_index);

static s8 nav_bit_quantize(s32 bit_integrate);
static update_count_t update_count_diff(tracker_channel_t *tracker_channel,
                                        update_count_t *val);

static void nap_channel_disable(u8 channel);
static void tracking_channel_ambiguity_unknown(u8 channel);
static void tracking_channel_get_corrs(u8 channel);
static void tracking_channel_process(u8 channel, bool update_required);
static void tracking_channel_update(u8 channel);

static bool track_iq_output_notify(struct setting *s, const char *val);

/** Initialize a nav_bit_fifo_t struct. */
static void nav_bit_fifo_init(nav_bit_fifo_t *fifo)
{
  fifo->read_index = 0;
  fifo->write_index = 0;
}

/** Write data to the nav bit FIFO.
 *
 * \note This function should only be called internally by the tracking thread.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 * \param element     Element to write to the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
static bool nav_bit_fifo_write(nav_bit_fifo_t *fifo,
                               const nav_bit_fifo_element_t *element)
{
  if (NAV_BIT_FIFO_LENGTH(fifo) < NAV_BIT_FIFO_SIZE) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    memcpy(&fifo->elements[fifo->write_index & NAV_BIT_FIFO_INDEX_MASK],
           element, sizeof(nav_bit_fifo_element_t));
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->write_index++;
    return true;
  }

  return false;
}

/** Read pending data from the nav bit FIFO.
 *
 * \note This function should only be called externally by the decoder thread.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 * \param element     Output element read from the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
static bool nav_bit_fifo_read(nav_bit_fifo_t *fifo,
                              nav_bit_fifo_element_t *element)
{
  if (NAV_BIT_FIFO_LENGTH(fifo) > 0) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    memcpy(element, &fifo->elements[fifo->read_index & NAV_BIT_FIFO_INDEX_MASK],
           sizeof(nav_bit_fifo_element_t));
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->read_index++;
    return true;
  }

  return false;
}

/** Initialize a nav_time_sync_t struct. */
static void nav_time_sync_init(nav_time_sync_t *sync)
{
  sync->valid = false;
}

/** Write pending time sync data from the decoder thread.
 *
 * \note This function should only be called externally by the decoder thread.
 *
 * \param sync          nav_time_sync_t struct to use.
 * \param TOW_ms        TOW in ms.
 * \param bit_polarity  Bit polarity.
 * \param read_index    Nav bit FIFO read index to which the above values
 *                      are synchronized.
 *
 * \return true if data was stored successfully, false otherwise.
 */
static bool nav_time_sync_set(nav_time_sync_t *sync, s32 TOW_ms,
                              s8 bit_polarity, nav_bit_fifo_index_t read_index)
{
  bool result = false;
  chMtxLock(&nav_time_sync_mutex);

  sync->TOW_ms = TOW_ms;
  sync->bit_polarity = bit_polarity;
  sync->read_index = read_index;
  sync->valid = true;
  result = true;

  Mutex *m = chMtxUnlock();
  assert(m == &nav_time_sync_mutex);
  return result;
}

/** Read pending time sync data provided by the decoder thread.
 *
 * \note This function should only be called internally by the tracking thread.
 *
 * \param sync          nav_time_sync_t struct to use.
 * \param TOW_ms        TOW in ms.
 * \param bit_polarity  Bit polarity.
 * \param read_index    Nav bit FIFO read index to which the above values
 *                      are synchronized.
 *
 * \return true if outputs are valid, false otherwise.
 */
static bool nav_time_sync_get(nav_time_sync_t *sync, s32 *TOW_ms,
                              s8 *bit_polarity, nav_bit_fifo_index_t *read_index)
{
  bool result = false;
  chMtxLock(&nav_time_sync_mutex);

  if (sync->valid) {
    *TOW_ms = sync->TOW_ms;
    *bit_polarity = sync->bit_polarity;
    *read_index = sync->read_index;
    sync->valid = false;
    result = true;
  }

  Mutex *m = chMtxUnlock();
  assert(m == &nav_time_sync_mutex);
  return result;
}

/** Compress a 32 bit integration value down to 8 bits. */
static s8 nav_bit_quantize(s32 bit_integrate)
{
  //  0 through  2^24 - 1 ->  0 = weakest positive bit
  // -1 through -2^24     -> -1 = weakest negative bit

  if (bit_integrate >= 0)
    return bit_integrate / (1 << 24);
  else
    return ((bit_integrate + 1) / (1 << 24)) - 1;
}

/** Returns the unsigned difference between update_count and *val for the
 * specified tracking channel.
 *
 * \note This function ensures that update_count is read prior to *val
 * such that erroneous jumps between zero and large positive numbers are
 * avoided.
 *
 * \param channel   Tracking channel to use.
 * \param val       Pointer to the value to be subtracted from update_count.
 *
 * \return The unsigned difference between update_count and *val.
 */
static update_count_t update_count_diff(tracker_channel_t *tracker_channel,
                                        update_count_t *val)
{
  tracker_common_data_t *common_data = tracker_channel->common_data;

  /* TODO: fix me: write to update count */
  /* Ensure that update_count is read prior to *val */
  update_count_t update_count = common_data->update_count;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return (update_count_t)(update_count - *val);
}

/** Disable a tracking channel in the NAP.
 * \param channel Tracking channel to disable.
 */
static void nap_channel_disable(u8 channel)
{
  nap_track_update_wr_blocking(channel, 0, 0, 0, 0);
}

/** Sets a channel's carrier phase ambiguity to unknown.
 * Changes the lock counter to indicate to the consumer of the tracking channel
 * observations that the carrier phase ambiguity may have changed. Also
 * invalidates the half cycle ambiguity, which must be resolved again by the navigation
 *  message processing. Should be called if a cycle slip is suspected.
 *
 * \param channel Tracking channel number to mark phase-ambiguous.
 */
static void tracking_channel_ambiguity_unknown(u8 channel)
{
  gnss_signal_t sid = tracking_channel[channel].sid;

  tracking_channel[channel].bit_polarity = BIT_POLARITY_UNKNOWN;
  tracking_channel[channel].lock_counter =
      ++tracking_lock_counters[sid_to_global_index(sid)];
}

/** Get correlations from a NAP tracking channel and store them in the
 * tracking channel state struct.
 * \param channel Tracking channel to read correlations for.
 */
static void tracking_channel_get_corrs(u8 channel)
{
  tracking_channel_t* chan = &tracking_channel[channel];

  /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
  if ((chan->int_ms > 1) && !chan->short_cycle) {
    /* If we just requested the short cycle, this is the long cycle's
     * correlations. */
    corr_t cs[3];
    nap_track_corr_rd_blocking(channel, &chan->corr_sample_count, cs);
    /* accumulate short cycle correlations with long */
    for(int i = 0; i < 3; i++) {
      chan->cs[i].I += cs[i].I;
      chan->cs[i].Q += cs[i].Q;
    }
  } else {
    nap_track_corr_rd_blocking(channel, &chan->corr_sample_count, chan->cs);
    alias_detect_first(&chan->alias_detect, chan->cs[1].I, chan->cs[1].Q);
  }
}

/** Checks the state of a tracking channel and generates events if required.
 * \param channel           Tracking channel to use.
 * \param update_required   True when correlations are pending for the
 *                          tracking channel.
 */
static void tracking_channel_process(u8 channel, bool update_required)
{
  tracking_channel_t* chan = &tracking_channel[channel];
  switch (state_get(chan)) {
    case STATE_ENABLED:
    {
      if (update_required) {
        tracking_channel_update(channel);
      }
      break;
    }
    case STATE_DISABLE_REQUESTED:
    {
      nap_channel_disable(channel);
      chan->disable_time = chTimeNow();
      event(chan, EVENT_DISABLE);
      break;
    }
    case STATE_DISABLED:
    {
      /* Tracking channel is not owned by the update thread, but the update
       * register must be written to clear the interrupt flag. Atomically
       * verify state and write the update register. */
      tracking_channel_lock(channel);
      if (state_get(chan) == STATE_DISABLED) {
        nap_channel_disable(channel);
      }
      tracking_channel_unlock(channel);
      break;
    }
    default:
    {
      assert(!"Invalid tracking channel state");
      break;
    }
  }
}

/** Update tracking channels after the end of an integration period.
 * Update update_count, sample_count, TOW, run loop filters and update
 * SwiftNAP tracking channel frequencies.
 * \param channel Tracking channel to update.
 */
static void tracking_channel_update(u8 channel)
{
  /* TODO: remove me */
}

/** Parse the IQ output enable bitfield. */
static bool track_iq_output_notify(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    for (int i = 0; i < NUM_TRACKER_CHANNELS; i++) {
      tracker_channel_t *tracker_channel = tracker_channel_get(i);
      tracker_internal_data_t *internal_data = tracker_channel->internal_data;
      internal_data.output_iq = (iq_output_mask & (1 << i)) != 0;
    }
    return true;
  }
  return false;
}

/** Send tracking state SBP message.
 * Send information on each tracking channel to host.
 */
void tracking_send_state()
{
  tracking_channel_state_t states[nap_track_n_channels];

  if (simulation_enabled_for(SIMULATION_MODE_TRACKING)) {

    u8 num_sats = simulation_current_num_sats();
    for (u8 i=0; i < num_sats; i++) {
      states[i] = simulation_current_tracking_state(i);
    }
    if (num_sats < nap_track_n_channels) {
      for (u8 i = num_sats; i < nap_track_n_channels; i++) {
        states[i].state = 0;
        states[i].sid.code = 0;
        states[i].sid.sat = 0;
        states[i].cn0 = -1;
      }
    }

  } else {

    for (u8 i=0; i<nap_track_n_channels; i++) {
      states[i].state = tracking_channel_running(i) ? 1 : 0;
      states[i].sid = sid_to_sbp(tracking_channel[i].sid);
      states[i].cn0 = tracking_channel_running(i) ?
                          tracking_channel[i].cn0 : -1;
    }
  }

  sbp_send_msg(SBP_MSG_TRACKING_STATE, sizeof(states), (u8*)states);
}

/** Force a satellite to drop.
 * This function is used for testing.  It clobbers the code frequency in the
 * loop filter which destroys the correlations.  The satellite is dropped
 * by manage.c which checks the SNR.
 */
void tracking_drop_satellite(gnss_signal_t sid)
{
  for (u8 i=0; i<nap_track_n_channels; i++) {
    if (!sid_is_equal(tracking_channel[i].sid, sid))
      continue;

    tracking_channel[i].tl_state.code_filt.y += 500;
  }
}

/** Calculate the future code phase after N samples.
 * Calculate the expected code phase in N samples time with carrier aiding.
 *
 * \param code_phase   Current code phase in chips.
 * \param carrier_freq Current carrier frequency (i.e. Doppler) in Hz used for
 *                     carrier aiding.
 * \param n_samples    N, the number of samples to propagate for.
 *
 * \return The propagated code phase in chips.
 */
float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples)
{
  /* Calculate the code phase rate with carrier aiding. */
  u32 code_phase_rate = (1.0 + carrier_freq/GPS_L1_HZ) * NAP_TRACK_NOMINAL_CODE_PHASE_RATE;

  /* Internal Swift NAP code phase is in chips*2^32:
   *
   * |  Chip no.  | Sub-chip | Fractional sub-chip |
   * | 0 ... 1022 | 0 ... 15 |  0 ... (2^28 - 1)   |
   *
   * Code phase rate is directly added in this representation,
   * the nominal code phase rate corresponds to 1 sub-chip.
   */

  /* Calculate code phase in chips*2^32. */
  u64 propagated_code_phase = (u64)(code_phase * (((u64)1)<<32)) + n_samples * (u64)code_phase_rate;

  /* Convert code phase back to natural units with sub-chip precision.
   * NOTE: the modulo is required to fix the fact rollover should
   * occur at 1023 not 1024.
   */
  return (float)((u32)(propagated_code_phase >> 28) % (1023*16)) / 16.0;
}

/** Initialises a tracking channel.
 * Initialises a tracking channel on the Swift NAP. The start_sample_count
 * must be contrived to be at or close to a PRN edge (PROMPT code phase = 0).
 *
 * \param prn                PRN number - 1 (0-31).
 * \param channel            Tracking channel number on the Swift NAP.
 * \param carrier_freq       Carrier frequency (Doppler) at start of tracking in Hz.
 * \param start_sample_count Sample count on which to start tracking.
 * \param cn0_init           Estimated C/N0 from acquisition
 * \param elevation          Satellite elevation in degrees, or
 *                           TRACKING_ELEVATION_UNKNOWN
 */
void tracking_channel_init(u8 channel, gnss_signal_t sid, float carrier_freq,
                           u32 start_sample_count, float cn0_init, s8 elevation)
{
  /* TODO: remove me */
}

/** Handles pending IRQs for the specified tracking channels.
 * \param channels_mask Bitfield indicating the tracking channels for which
 *                      an IRQ is pending.
 */
void tracking_channels_update(u32 channels_mask)
{
  /* For each tracking channel, call tracking_channel_process(). Indicate
   * that an update is required if the corresponding bit is set in
   * channels_mask.
   */
  for (u32 channel = 0; channel < nap_track_n_channels; channel++) {
    bool update_required = (channels_mask & 1) ? true : false;
    tracking_channel_process(channel, update_required);
    channels_mask >>= 1;
  }
}

/** Locks a tracking channel for exclusive access.
 * \note Blocking or long-running operations should not be performed while
 *       holding the lock for a tracking channel.
 * \param channel Tracking channel to lock.
 */
void tracking_channel_lock(u8 channel)
{
  /* TODO: remove me */
}

/** Unlocks a locked tracking channel.
 * \param channel Tracking channel to unlock.
 */
void tracking_channel_unlock(u8 channel)
{
  /* TODO: remove me */
}

/** Determines whether the specified tracking channel is currently running.
 * \param channel Tracking channel to use.
 */
bool tracking_channel_running(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  return (tracker_channel_state_get(tracker_channel) == STATE_ENABLED);
}

/** Determines if C/NO is above the use threshold for a tracking channel.
 * \param channel Tracking channel to use.
 */
bool tracking_channel_cn0_useable(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_common_data_t *common_data = tracker_channel->common_data;
  //return (common_data->cn0 > track_cn0_use_thres);
  /* TODO: fix me */
  return true;
}

/** Returns the time in ms for which tracking channel has been running.
 * \param channel Tracking channel to use.
 */
u32 tracking_channel_running_time_ms_get(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_common_data_t *common_data = tracker_channel->common_data;
  return common_data->update_count;
}

/** Returns the time in ms for which C/N0 has been above the use threshold for
 * a tracking channel.
 * \param channel Tracking channel to use.
 */
u32 tracking_channel_cn0_useable_ms_get(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_common_data_t *common_data = tracker_channel->common_data;
  return update_count_diff(channel,
      &common_data->cn0_below_use_thres_count);
}

/** Returns the time in ms for which C/N0 has been below the drop threshold for
 * a tracking channel.
 * \param channel Tracking channel to use.
 */
u32 tracking_channel_cn0_drop_ms_get(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_common_data_t *common_data = tracker_channel->common_data;
  return update_count_diff(channel,
      &common_data->cn0_above_drop_thres_count);
}

/** Returns the time in ms for which the optimistic lock detector has reported
 * being unlocked for a tracking channel.
 * \param channel Tracking channel to use.
 */
u32 tracking_channel_ld_opti_unlocked_ms_get(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_common_data_t *common_data = tracker_channel->common_data;
  return update_count_diff(channel,
      &common_data->ld_opti_locked_count);
}

/** Returns the time in ms for which the pessimistic lock detector has reported
 * being locked for a tracking channel.
 * \param channel Tracking channel to use.
 */
u32 tracking_channel_ld_pess_locked_ms_get(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_common_data_t *common_data = tracker_channel->common_data;
  return update_count_diff(channel,
      &common_data->ld_pess_unlocked_count);
}

/** Returns the time in ms since the last mode change for a tracking channel.
 * \param channel Tracking channel to use.
 */
u32 tracking_channel_last_mode_change_ms_get(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_common_data_t *common_data = tracker_channel->common_data;
  return update_count_diff(channel,
      &common_data->mode_change_count);
}

/** Returns the sid currently associated with a tracking channel.
 * \param channel Tracking channel to use.
 */
gnss_signal_t tracking_channel_sid_get(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  return tracker_channel->info.sid;
}

/** Returns the current carrier frequency for a tracking channel.
 * \param channel Tracking channel to use.
 */
double tracking_channel_carrier_freq_get(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_common_data_t *common_data = tracker_channel->common_data;
  return common_data->carrier_freq;
}

/** Returns the current time of week for a tracking channel.
 * \param channel Tracking channel to use.
 */
s32 tracking_channel_tow_ms_get(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_common_data_t *common_data = tracker_channel->common_data;
  return common_data->TOW_ms;
}

/** Returns the bit sync status for a tracking channel.
 * \param channel Tracking channel to use.
 */
bool tracking_channel_bit_sync_resolved(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_internal_data_t *internal_data = tracker_channel->internal_data;
  return (internal_data->bit_sync.bit_phase_ref != BITSYNC_UNSYNCED);
}

/** Returns the bit polarity resolution status for a tracking channel.
 * \param channel Tracking channel to use.
 */
bool tracking_channel_bit_polarity_resolved(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_internal_data_t *internal_data = tracker_channel->internal_data;
  return (internal_data->bit_polarity != BIT_POLARITY_UNKNOWN);
}

/** Update channel measurement for a tracking channel.
 * \param channel Tracking channel to update measurement from.
 * \param meas Pointer to channel_measurement_t where measurement will be put.
 */
void tracking_channel_measurement_get(u8 channel, channel_measurement_t *meas)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_internal_data_t *internal_data = tracker_channel->internal_data;
  tracker_common_data_t *common_data = tracker_channel->common_data;

  /* Update our channel measurement. */
  meas->sid = tracker_channel->info.sid;
  meas->code_phase_chips = (double)common_data->code_phase_early /
                               NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  meas->code_phase_rate = common_data->code_phase_rate;
  meas->carrier_phase = common_data->carrier_phase / (double)(1<<24);
  meas->carrier_freq = common_data->carrier_freq;
  meas->time_of_week_ms = common_data->TOW_ms;
  meas->receiver_time = (double)common_data->sample_count / SAMPLE_FREQ;
  meas->snr = common_data->cn0;
  if (internal_data->bit_polarity == BIT_POLARITY_INVERTED) {
    meas->carrier_phase += 0.5;
  }
  meas->lock_counter = internal_data->lock_counter;
}

/** Sets the elevation angle (deg) for a tracking channel by sid.
 * \param sid         Signal identifier for which the elevation should be set.
 * \param elevation   Elevation angle in degrees.
 */
bool tracking_channel_evelation_degrees_set(gnss_signal_t sid, s8 elevation)
{
  bool result = false;
  for (u32 i=0; i < NUM_TRACKER_CHANNELS; i++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(i);
    tracker_internal_data_t *internal_data = tracker_channel->internal_data;

    /* Check SID before locking. */
    if (!sid_is_equal(tracker_channel->info.sid, sid)) {
      continue;
    }

    /* Lock and update if SID matches. */
    //tracking_channel_lock(i);
    if (sid_is_equal(tracker_channel->info.sid, sid)) {
      internal_data->elevation = elevation;
      result = true;
    }
    //tracking_channel_unlock(i);
    break;
  }
  return result;
}

/** Returns the elevation angle (deg) for a tracking channel.
 * \param channel Tracking channel to use.
 */
s8 tracking_channel_evelation_degrees_get(u8 channel)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(channel);
  tracker_internal_data_t *internal_data = tracker_channel->internal_data;
  return internal_data->elevation;
}

/** Read the next pending nav bit for a tracking channel.
 *
 * \note This function should should be called from the same thread as
 * tracking_channel_time_sync().
 *
* \param tracker_channel_id   ID of tracker channel to read from.
 * \param soft_bit            Output soft nav bit value.
 *
 * \return true if *soft_bit is valid, false otherwise.
 */
bool tracking_channel_nav_bit_get(tracker_channel_id_t tracker_channel_id,
                                  s8 *soft_bit)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(tracker_channel_id);
  tracker_internal_data_t *internal_data = tracker_channel->internal_data;

  nav_bit_fifo_element_t element;
  if (nav_bit_fifo_read(&internal_data->nav_bit_fifo, &element)) {
    *soft_bit = element.soft_bit;
    return true;
  }
  return false;
}

/** Propagate decoded time of week and bit polarity back to a tracking channel.
 *
 * \note This function should be called from the same thread as
 * tracking_channel_nav_bit_get().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracking_channel_nav_bit_get().
 *
 * \param tracker_channel_id    ID of tracker channel to synchronize.
 * \param TOW_ms                Time of week in milliseconds.
 * \param bit_polarity  Bit polarity.
 *
 * \return true if data was enqueued successfully, false otherwise.
 */
bool tracking_channel_time_sync(tracker_channel_id_t tracker_channel_id,
                                s32 TOW_ms, s8 bit_polarity)
{
  assert(TOW_ms >= 0);
  assert(TOW_ms < GPS_WEEK_LENGTH_ms);
  assert((bit_polarity == BIT_POLARITY_NORMAL) ||
         (bit_polarity == BIT_POLARITY_INVERTED));

  tracker_channel_t *tracker_channel = tracker_channel_get(tracker_channel_id);
  tracker_internal_data_t *internal_data = tracker_channel->internal_data;
  nav_bit_fifo_index_t read_index = internal_data->nav_bit_fifo.read_index;
  return nav_time_sync_set(&internal_data->nav_time_sync,
                           TOW_ms, bit_polarity, read_index);
}

/** \} */











/** Set up the tracking module. */
void track_setup(void)
{
  SETTING_NOTIFY("track", "iq_output_mask", iq_output_mask, TYPE_INT,
                 track_iq_output_notify);

  for (u32 i=0; i < PLATFORM_SIGNAL_COUNT; i++) {
    tracking_lock_counters[i] = random_int();
  }

  for (u32 i=0; i<NUM_TRACKER_CHANNELS; i++) {
    tracker_channels[i].state = STATE_DISABLED;
    tracker_channels[i].tracker = 0;
    chMtxInit(&tracker_channels[i].common_data_mutex);
    chMtxInit(&tracker_channels[i].nap_mutex);
  }

  /*
  chThdCreateStatic(wa_track_thread, sizeof(wa_track_thread),
                    NORMALPRIO-1, track_thread, NULL);
   */
}

/** Register a tracker interface to enable tracking for a code type.
 *
 * \note element and all subordinate data must be statically allocated!
 *
 * \param element   Struct describing the interface to register.
 */
void tracker_interface_register(tracker_interface_list_element_t *element)
{
  /* p_next = address of next pointer which must be updated */
  tracker_interface_list_element_t **p_next = &tracker_interface_list;

  while (*p_next != 0)
    p_next = &(*p_next)->next;

  element->next = 0;
  *p_next = element;
}

/** Determine if a tracker channel is available for the specified sid.
 *
 * \param sid       Signal to be tracked.
 *
 * \return true if a tracker channel is available, false otherwise.
 */
bool tracker_channel_available(gnss_signal_t sid)
{
  tracker_channel_t *tracker_channel;
  if(!available_tracker_channel_get(&tracker_channel))
    return false;

  const tracker_interface_t *interface = tracker_interface_get(sid);
  tracker_t *tracker;
  if (!available_tracker_get(interface, &tracker))
    return false;

  return true;
}

/** Initialize a tracker channel to track the specified sid.
 *
 * \param sid       Signal to be tracked.
 *
 * \return ID of initialized tracker channel or TRACKER_CHANNEL_ID_INVALID.
 */
tracker_channel_id_t tracker_channel_init(gnss_signal_t sid)
{
  /* TODO: fix me */
  float carrier_freq = 0;
  float code_phase_rate_fp = 0;
  u32 start_sample_count = 0;



  tracker_channel_t *tracker_channel;
  if(!available_tracker_channel_get(&tracker_channel))
    return TRACKER_CHANNEL_ID_INVALID;

  const tracker_interface_t *interface = tracker_interface_get(sid);
  tracker_t *tracker;
  if (!available_tracker_get(interface, &tracker))
    return TRACKER_CHANNEL_ID_INVALID;



  /* TODO: FIX HACK */
  tracker_channel_id_t id = (((u32)tracker_channel - (u32)tracker_channels) /
                            sizeof(tracker_channels[0]));


  /* Set up channel */
  tracker_channel->info.sid = sid;
  tracker_channel->tracker = tracker;

  /* TODO: mutex lock common data? */
  common_data_init(&tracker_channel->common_data);
  internal_data_init(&tracker_channel->internal_data);
  interface_function(tracker_channel, interface->init);



  /* Lock the NAP mutex while setting up NAP registers and updating state. This
   * allows the update thread to deal with trailing interrupts after the
   * channel is disabled by writing the update register as required.  */
  chMtxLock(&tracker_channel->nap_mutex);

  /* Starting carrier phase is set to zero as we don't
   * know the carrier freq well enough to calculate it.
   */
  /* Start with code phase of zero as we have conspired for the
   * channel to be initialised on an EARLY code phase rollover.
   */
  nap_track_code_wr_blocking(id, sid);
  nap_track_init_wr_blocking(id, 0, 0, 0);
  nap_track_update_wr_blocking(
    id,
    carrier_freq*NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ,
    code_phase_rate_fp,
    0, 0
  );

  /* Schedule the timing strobe for start_sample_count. */
  nap_timing_strobe(start_sample_count);

  /* Change the channel state to ENABLED. */
  event(tracker_channel, EVENT_ENABLE);

  Mutex *m = chMtxUnlock();
  assert(m == &tracker_channel->nap_mutex);

  return id;
}

/** Disable the specified tracker channel.
 *
 * \param tracker_channel_id    ID of tracker channel to be disabled.
 *
 * \return true if a tracker channel was disabled, false otherwise.
 */
bool tracker_channel_disable(tracker_channel_id_t tracker_channel_id)
{
  /* Request disable */
  tracker_channel_t *tracker_channel = tracker_channel_get(tracker_channel_id);
  event(tracker_channel, EVENT_DISABLE_REQUEST);
  return true;
}













/** Retrieve the tracker channel associated with the specified
 * tracker channel ID.
 *
 * \param tracker_channel_id    ID of tracker channel to be retrieved.
 *
 * \return Associated tracker channel.
 */
static tracker_channel_t * tracker_channel_get(tracker_channel_id_t
                                               tracker_channel_id)
{
  assert(tracker_channel_id < NUM_TRACKER_CHANNELS);
  return &tracker_channels[tracker_channel_id];
}

/** Retrieve the tracker interface for the specified sid.
 *
 * \param sid       Signal to be tracked.
 *
 * \return Associated tracker interface. May be the default interface.
 */
static const tracker_interface_t * tracker_interface_get(gnss_signal_t sid)
{
  const tracker_interface_list_element_t *e = tracker_interface_list;
  while (e != 0) {
    const tracker_interface_t *interface = e->interface;
    if (interface->code == sid.code) {
      return interface;
    }
    e = e->next;
  }

  return &tracker_interface_default;
}

/** Find an inactive tracker channel.
 *
 * \param tracker_channel   Output inactive tracker channel.
 *
 * \return true if *tracker_channel points to an inactive tracker channel,
 * false otherwise.
 */
static bool available_tracker_channel_get(tracker_channel_t **tracker_channel)
{
  for (u32 i=0; i<NUM_TRACKER_CHANNELS; i++) {
    tracker_channel_t *tc = &tracker_channels[i];
    if (tracker_channel_state_get(tc) == STATE_DISABLED) {
      if (chTimeElapsedSince(tc->internal_data.disable_time) >=
            MS2ST(CHANNEL_SHUTDOWN_TIME_ms)) {
        *tracker_channel = tc;
        return true;
      }
    }
  }

  return false;
}

/** Find an inactive tracker instance for the specified tracker interface.
 *
 * \param interface       Tracker interface to use.
 * \param tracker         Output inactive tracker instance.
 *
 * \return true if *tracker points to an inactive tracker instance,
 * false otherwise.
 */
static bool available_tracker_get(const tracker_interface_t *interface,
                                  tracker_t **tracker)
{
  /* Search for a free tracker */
  for (u32 i=0; i<interface->num_trackers; i++) {
    tracker_t *t = &interface->trackers[i];
    if (!tracker_active(t)) {
      *tracker = t;
      return true;
    }
  }

  return false;
}

/** Return the state of a tracker channel.
 *
 * \note This function performs an acquire operation, meaning that it ensures
 * the returned state was read before any subsequent memory accesses.
 *
 * \param tracker_channel   Tracker channel to use.
 *
 * \return state of the decoder channel.
 */
static state_t tracker_channel_state_get(const tracker_channel_t *
                                         tracker_channel)
{
  state_t state = tracker_channel->state;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return state;
}

/** Return the state of a tracker instance.
 *
 * \note This function performs an acquire operation, meaning that it ensures
 * the returned state was read before any subsequent memory accesses.
 *
 * \param tracker   Tracker to use.
 *
 * \return true if the tracker is active, false if inactive.
 */
static bool tracker_active(const tracker_t *tracker)
{
  bool active = tracker->active;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return active;
}

/** Execute an interface function on a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param func              Interface function to execute.
 */
static void interface_function(tracker_channel_t *tracker_channel,
                               tracker_interface_function_t func)
{
  return func(&tracker_channel->info, &tracker_channel->common_data,
              tracker_channel->tracker->data);
}

/** Update the state of a tracker channel and its associated tracker instance.
 *
 * \note This function performs a release operation, meaning that it ensures
 * all prior memory accesses have completed before updating state information.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param event             Event to process.
 */
static void event(tracker_channel_t *tracker_channel, event_t event)
{
  switch (event) {
  case EVENT_ENABLE: {
    assert(tracker_channel->state == STATE_DISABLED);
    assert(tracker_channel->tracker->active == false);
    tracker_channel->tracker->active = true;
    /* Sequence point for enable is setting channel state = STATE_ENABLED */
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    tracker_channel->state = STATE_ENABLED;
  }
  break;

  case EVENT_DISABLE_REQUEST: {
    assert(tracker_channel->state == STATE_ENABLED);
    tracker_channel->state = STATE_DISABLE_REQUESTED;
  }
  break;

  case EVENT_DISABLE: {
    assert(tracker_channel->state == STATE_DISABLE_REQUESTED);
    assert(tracker_channel->tracker->active == true);
    /* Sequence point for disable is setting channel state = STATE_DISABLED
     * and/or tracker active = false (order of these two is irrelevant here) */
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    tracker_channel->tracker->active = false;
    tracker_channel->state = STATE_DISABLED;
  }
  break;
  }
}





static void common_data_init(tracker_common_data_t *common_data)
{
  /* TODO: fix me */
  float carrier_freq = 0;
  u32 start_sample_count = 0;

  /* Initialize all fields to 0 */
  memset(common_data, 0, sizeof(tracker_common_data_t));

  common_data->TOW_ms = TOW_INVALID;

  /* Calculate code phase rate with carrier aiding. */
  common_data->code_phase_rate = (1 + carrier_freq/GPS_L1_HZ) * GPS_CA_CHIPPING_RATE;
  common_data->carrier_freq = carrier_freq;

  /* Adjust the channel start time as the start_sample_count passed
   * in corresponds to a PROMPT code phase rollover but we want to
   * start the channel on an EARLY code phase rollover.
   */
  /* TODO : change hardcoded sample rate */
  common_data->sample_count = start_sample_count - 0.5*16;
}

static void internal_data_init(tracker_internal_data_t *internal_data)
{
  /* TODO: fix me */
  u8 elevation = 0;
  gnss_signal_t sid = {0};

  /* Initialize all fields to 0 */
  memset(internal_data, 0, sizeof(tracker_internal_data_t));

  internal_data->elevation = elevation;

  internal_data->bit_polarity = BIT_POLARITY_UNKNOWN;

  nav_bit_fifo_init(&internal_data->nav_bit_fifo);
  nav_time_sync_init(&internal_data->nav_time_sync);
  bit_sync_init(&internal_data->bit_sync, sid);
}


static tracker_channel_t *current_tracker_channel = 0;
void tracker_common_data_update(const tracker_common_data_t *common_data)
{
  chMtxLock(&current_tracker_channel->common_data_mutex);
  current_tracker_channel->common_data = *common_data;
  Mutex *m = chMtxUnlock();
  assert(m == &current_tracker_channel->common_data_mutex);
}

s32 tracker_tow_update(u32 int_ms)
{
  tracker_channel_info_t *channel_info = current_tracker_channel->info;
  tracker_internal_data_t *internal_data = current_tracker_channel->internal_data;
  tracker_common_data_t *common_data = current_tracker_channel->common_data;

  char buf[SID_STR_LEN_MAX];
  sid_to_string(buf, sizeof(buf), channel_info->sid);

  s32 channel_TOW_ms = common_data->TOW_ms;

  /* Latch TOW from nav message if pending */
  s32 pending_TOW_ms;
  s8 pending_bit_polarity;
  nav_bit_fifo_index_t pending_TOW_read_index;
  if (nav_time_sync_get(&internal_data->nav_time_sync, &pending_TOW_ms,
                        &pending_bit_polarity, &pending_TOW_read_index)) {

    /* Compute time since the pending data was read from the FIFO */
    nav_bit_fifo_index_t fifo_length =
      NAV_BIT_FIFO_INDEX_DIFF(internal_data->nav_bit_fifo.write_index,
                              pending_TOW_read_index);
    u32 fifo_time_diff_ms = fifo_length * internal_data->bit_sync.bit_length;

    /* Add full bit times + fractional bit time to the specified TOW */
    s32 TOW_ms = pending_TOW_ms + fifo_time_diff_ms +
                   internal_data->nav_bit_TOW_offset_ms;

    if (TOW_ms >= GPS_WEEK_LENGTH_ms)
      TOW_ms -= GPS_WEEK_LENGTH_ms;

    /* Warn if updated TOW does not match the current value */
    if ((channel_TOW_ms != TOW_INVALID) && (channel_TOW_ms != TOW_ms)) {
      log_warn("%s TOW mismatch: %ld, %lu", buf, channel_TOW_ms, TOW_ms);
    }
    channel_TOW_ms = TOW_ms;
    internal_data->bit_polarity = pending_bit_polarity;
  }

  internal_data->nav_bit_TOW_offset_ms += int_ms;

  if (channel_TOW_ms != TOW_INVALID) {
    /* Have a valid time of week - increment it. */
    channel_TOW_ms += int_ms;
    if (channel_TOW_ms >= GPS_WEEK_LENGTH_ms)
      channel_TOW_ms -= GPS_WEEK_LENGTH_ms;
    /* TODO: maybe keep track of week number in channel state, or
       derive it from system time */
  }

  return channel_TOW_ms;
}

void tracker_bit_sync_update(u32 int_ms, s32 corr_prompt_real)
{
  tracker_channel_info_t *channel_info = current_tracker_channel->info;
  tracker_internal_data_t *internal_data = current_tracker_channel->internal_data;

  char buf[SID_STR_LEN_MAX];
  sid_to_string(buf, sizeof(buf), channel_info->sid);

  /* Update bit sync */
  s32 bit_integrate;
  if (bit_sync_update(&internal_data->bit_sync, corr_prompt_real, int_ms,
                      &bit_integrate)) {

    s8 soft_bit = nav_bit_quantize(bit_integrate);

    // write to FIFO
    nav_bit_fifo_element_t element = { .soft_bit = soft_bit };
    if (!nav_bit_fifo_write(&internal_data->nav_bit_fifo, &element)) {
      log_warn("%s nav bit FIFO overrun", buf);
    }

    // clear nav bit TOW offset
    internal_data->nav_bit_TOW_offset_ms = 0;
  }
}

bool tracker_channel_bit_aligned(void)
{
  tracker_internal_data_t *internal_data = current_tracker_channel->internal_data;

  return (internal_data->bit_sync.bit_phase ==
            internal_data->bit_sync.bit_phase_ref);
}

void tracker_channel_correlations_send(const corr_t *cs)
{
  /* TODO: fix me */
  u8 channel = 0;

  tracker_channel_info_t *channel_info = current_tracker_channel->info;
  tracker_internal_data_t *internal_data = current_tracker_channel->internal_data;

  /* Output I/Q correlations using SBP if enabled for this channel */
  if (internal_data->output_iq) {
    msg_tracking_iq_t msg = {
      .channel = channel,
    };
    msg.sid = sid_to_sbp(channel_info->sid);
    for (u32 i = 0; i < 3; i++) {
      msg.corrs[i].I = cs[i].I;
      msg.corrs[i].Q = cs[i].Q;
    }
    sbp_send_msg(SBP_MSG_TRACKING_IQ, sizeof(msg), (u8*)&msg);
  }
}



