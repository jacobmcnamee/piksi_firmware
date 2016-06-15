/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "peripherals/usart.h"

void usart_support_init(void)
{
}

void usart_support_set_parameters(void *sd, u32 baud)
{
  (void)sd;
  (void)baud;
}

void usart_support_disable(void *sd)
{
  (void)sd;
}

u32 usart_support_n_read(void *sd)
{
  (void)sd;
  return 0;
}

u32 usart_support_tx_n_free(void *sd)
{
  (void)sd;
  return 0;
}

u32 usart_support_read_timeout(void *sd, u8 data[], u32 len, u32 timeout)
{
  (void)sd;
  (void)data;
  (void)len;
  (void)timeout;
  return 0;
}

u32 usart_support_write(void *sd, const u8 data[], u32 len)
{
  (void)sd;
  (void)data;
  (void)len;
  return 0;
}

