/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/gpio.h>

void gpio_setup(void)
{
  RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3|GPIO4);
}

int main(void)
{
	int i;

	gpio_setup();

  gpio_set(GPIOC, GPIO3);
  gpio_clear(GPIOC, GPIO4);

	/* Blink the LEDs on the board. */
	while (1) {
		/* Using API function gpio_toggle(): */
		gpio_toggle(GPIOC, GPIO3);	/* LED on/off */
		gpio_toggle(GPIOC, GPIO4);	/* LED on/off */
		for (i = 0; i < 600000; i++)	/* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}
