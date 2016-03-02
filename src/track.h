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

#ifndef SWIFTNAV_TRACK_H
#define SWIFTNAV_TRACK_H

#include <libsbp/tracking.h>
#include <libswiftnav/common.h>
#include <libswiftnav/nav_msg.h>
#include <libswiftnav/track.h>
#include <libswiftnav/signal.h>

#include <ch.h>

#include "board/nap/track_channel.h"

/** \addtogroup tracking
 * \{ */

#define TRACKING_ELEVATION_UNKNOWN 100 /* Ensure it will be above elev. mask */

/** \} */

void tracking_send_state(void);
void tracking_drop_satellite(gnss_signal_t sid);

float propagate_code_phase(float code_phase, float carrier_freq, u32 n_samples);

/* State management interface */
void tracking_channel_init(u8 channel, gnss_signal_t sid, float carrier_freq,
                           u32 start_sample_count, float cn0_init, s8 elevation);

/* Update interface */
void tracking_channels_update(u32 channels_mask);

/* Tracking parameters interface.
 * Lock should be acquired for atomicity. */
void tracking_channel_lock(u8 channel);
void tracking_channel_unlock(u8 channel);

bool tracking_channel_running(u8 channel);
bool tracking_channel_cn0_useable(u8 channel);
u32 tracking_channel_running_time_ms_get(u8 channel);
u32 tracking_channel_cn0_useable_ms_get(u8 channel);
u32 tracking_channel_cn0_drop_ms_get(u8 channel);
u32 tracking_channel_ld_opti_unlocked_ms_get(u8 channel);
u32 tracking_channel_ld_pess_locked_ms_get(u8 channel);
u32 tracking_channel_last_mode_change_ms_get(u8 channel);
gnss_signal_t tracking_channel_sid_get(u8 channel);
double tracking_channel_carrier_freq_get(u8 channel);
s32 tracking_channel_tow_ms_get(u8 channel);
bool tracking_channel_bit_sync_resolved(u8 channel);
bool tracking_channel_bit_polarity_resolved(u8 channel);
void tracking_channel_measurement_get(u8 channel, channel_measurement_t *meas);

bool tracking_channel_evelation_degrees_set(gnss_signal_t sid, s8 elevation);
s8 tracking_channel_evelation_degrees_get(u8 channel);

/* Decoder interface */
bool tracking_channel_nav_bit_get(u8 channel, s8 *soft_bit);
bool tracking_channel_time_sync(u8 channel, s32 TOW_ms, s8 bit_polarity);



typedef u32 update_count_t;

typedef struct {
  update_count_t update_count; /**< Number of ms channel has been running */
  update_count_t mode_change_count;
                               /**< update_count at last mode change. */
  update_count_t cn0_below_use_thres_count;
                               /**< update_count value when SNR was
                                  last below the use threshold. */
  update_count_t cn0_above_drop_thres_count;
                               /**< update_count value when SNR was
                                  last above the drop threshold. */
  update_count_t ld_opti_locked_count;
                               /**< update_count value when optimistic
                                  phase detector last "locked". */
  update_count_t ld_pess_unlocked_count;
                               /**< update_count value when pessimistic
                                  phase detector last "unlocked". */
  s32 TOW_ms;                  /**< TOW in ms. */

  u32 sample_count;            /**< Total num samples channel has tracked for. */
  u32 code_phase_early;        /**< Early code phase. */
  double code_phase_rate;      /**< Code phase rate in chips/s. */
  s64 carrier_phase;           /**< Carrier phase in NAP register units. */
  double carrier_freq;         /**< Carrier frequency Hz. */
  float cn0;                   /**< Current estimate of C/N0. */
} tracker_common_data_t;

typedef void tracker_data_t;

/** Instance of a tracker implementation. */
typedef struct {
  /** true if tracker is in use. */
  bool active;
  /** Pointer to implementation-specific data used by tracker instance. */
  tracker_data_t *data;
} tracker_t;

/** Info associated with a tracker channel. */
typedef struct {
  gnss_signal_t sid;    /**< Current signal being decoded. */
} tracker_channel_info_t;

/** Tracker interface function template. */
typedef void (*tracker_interface_function_t)(
                 const tracker_channel_info_t *channel_info,
                 const tracker_common_data_t *common_data,
                 tracker_data_t *tracker_data);

/** Interface to a tracker implementation. */
typedef struct {
  /** Code type for which the implementation may be used. */
  enum code code;
  /** Init function. Called to set up tracker instance when decoding begins. */
  tracker_interface_function_t init;
  /** Disable function. Called when tracking stops. */
  tracker_interface_function_t disable;
  /** Update function. Called when new correlation outputs are available. */
  tracker_interface_function_t update;
  /** Array of tracker instances used by this interface. */
  tracker_t *trackers;
  /** Number of tracker instances in trackers array. */
  u8 num_trackers;
} tracker_interface_t;

/** List element passed to tracker_interface_register(). */
typedef struct tracker_interface_list_element_t {
  const tracker_interface_t *interface;
  struct tracker_interface_list_element_t *next;
} tracker_interface_list_element_t;

typedef u8 tracker_channel_id_t;
#define TRACKER_CHANNEL_ID_INVALID 0xff

void track_setup(void);
void tracker_interface_register(tracker_interface_list_element_t *element);
bool tracker_channel_available(gnss_signal_t sid);
tracker_channel_id_t tracker_channel_init(gnss_signal_t sid);
bool tracker_channel_disable(tracker_channel_id_t id);

void tracker_common_data_update(const tracker_common_data_t *common_data);
s32 tracker_tow_update(u32 int_ms);
void tracker_bit_sync_update(u32 int_ms, s32 corr_prompt_real);
bool tracker_channel_bit_aligned(void);
void tracker_channel_correlations_send(const corr_t *cs);

#endif
