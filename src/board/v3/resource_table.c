/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <openamp/open_amp.h>
#include "resource_table.h"

#define RPMSG_IPU_C0_FEATURES       1
#define VIRTIO_RPMSG_F_NS           0

/* Resource table entries */
#define ELF_START                   0x1E000000
#define ELF_SIZE                    0x01000000
#define NUM_VRINGS                  2
#define VRING_ALIGN                 0x00001000
#define VRING_SIZE                  0x00010000
#define NUM_TABLE_ENTRIES           2

const struct remote_resource_table __attribute__((section (".resource_table")))
resources = {
  .version = 1,
  .num = NUM_TABLE_ENTRIES,
  .reserved = {0, 0},
  .offset = {
    offsetof(struct remote_resource_table, elf_cout),
    offsetof(struct remote_resource_table, rpmsg_vdev)
  },
  .elf_cout = {
    .type = RSC_CARVEOUT,
    .da = ELF_START,
    .pa = ELF_START,
    .len = ELF_SIZE,
    .flags = 0,
    .reserved = 0,
    .name = "ELF_COUT"
  },
  .rpmsg_vdev = {
    .type = RSC_VDEV,
    .id = VIRTIO_ID_RPMSG,
    .notifyid = 0,
    .dfeatures = RPMSG_IPU_C0_FEATURES,
    .gfeatures = 0,
    .config_len = 0,
    .status = 0,
    .num_of_vrings = NUM_VRINGS,
    .reserved = {0, 0}
  },
  .rpmsg_vring0 = {
    .da = 0,
    .align = VRING_ALIGN,
    .num = VRING_SIZE,
    .notifyid = 0,
    .reserved = 0
  },
  .rpmsg_vring1 = {
    .da = 0,
    .align = VRING_ALIGN,
    .num = VRING_SIZE,
    .notifyid = 0,
    .reserved = 0
  }
};
