/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/**
 * @file adc.c
 *
 * Board ADC on the io-channels of the px4,adc devicetree node, with the Zephyr ADC API.
 */

#include <drivers/drv_adc.h>
#include <px4_platform_common/log.h>

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>

#define ADC_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(px4_adc)

#define ADC_CHANNEL(node, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node, idx),

static const struct adc_dt_spec channels[] = {DT_FOREACH_PROP_ELEM(ADC_NODE, io_channels, ADC_CHANNEL)};

/* ADC_CHANNELS and the channel macros of the names, see platforms/zephyr/cmake/px4_impl_os.cmake */
BUILD_ASSERT(ADC_CHANNELS == BIT_MASK(ARRAY_SIZE(channels)), "ADC_CHANNELS isn't the channels of px4,adc");

static float reference_v;
static uint32_t full_count;

int px4_arch_adc_init(uint32_t base_address)
{
	int32_t full_scale_uv = 0;

	for (unsigned i = 0; i < ARRAY_SIZE(channels); i++) {
		const struct adc_dt_spec *channel = &channels[i];

		if (!adc_is_ready_dt(channel)) {
			PX4_ERR("channel %u: %s not ready", i, channel->dev->name);
			return -ENODEV;
		}

		int ret = adc_channel_setup_dt(channel);

		if (ret != 0) {
			PX4_ERR("channel %u: setup failed (%d)", i, ret);
			return ret;
		}

		/* the input voltage of the full count, from the reference voltage and the gain */
		int32_t channel_full_scale_uv = BIT(channel->resolution);
		ret = adc_raw_to_microvolts_dt(channel, &channel_full_scale_uv);

		if (ret != 0) {
			PX4_ERR("channel %u: no reference voltage (%d)", i, ret);
			return ret;
		}

		/* adc_report has one reference voltage and one resolution for all the channels */
		if ((i > 0) && ((channel_full_scale_uv != full_scale_uv) || (channel->resolution != channels[0].resolution))) {
			PX4_ERR("channel %u: full scale or resolution differs from channel 0", i);
			return -EINVAL;
		}

		full_scale_uv = channel_full_scale_uv;
	}

	reference_v = full_scale_uv * 1e-6f;
	full_count = BIT(channels[0].resolution);

	return 0;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	if (channel >= ARRAY_SIZE(channels)) {
		return UINT32_MAX;
	}

	/* drivers write 16-bit samples, or 32-bit ones for some ADCs of more than 16 bits (little-endian) */
	uint32_t sample = 0;
	struct adc_sequence sequence = {
		.buffer = &sample,
		.buffer_size = sizeof(sample),
	};

	adc_sequence_init_dt(&channels[channel], &sequence);

	if (adc_read_dt(&channels[channel], &sequence) != 0) {
		return UINT32_MAX;
	}

	return sample;
}

float px4_arch_adc_reference_v(void)
{
	return reference_v;
}

uint32_t px4_arch_adc_temp_sensor_mask(void)
{
	return 0;
}

uint32_t px4_arch_adc_dn_fullcount(void)
{
	return full_count;
}
