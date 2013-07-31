/*
 * Copyright (c) 2013 Seppo Tomperi
 *
 * This file is part of Libav.
 *
 * Libav is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * Libav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdint.h>

#include "libavutil/attributes.h"
#include "libavutil/arm/cpu.h"
#include "libavcodec/hevcdsp.h"

void ff_hevc_put_unweighted_pred_arm(uint8_t *_dst, ptrdiff_t _dststride,
        int16_t *src, ptrdiff_t srcstride, int width, int height);

static av_cold void h264dsp_init_neon(HEVCDSPContext *c, const int bit_depth)
{
    c->put_unweighted_pred = ff_hevc_put_unweighted_pred_arm;
}

av_cold void ff_hevcdsp_init_arm(HEVCDSPContext *c, const int bit_depth)
{
    int cpu_flags = av_get_cpu_flags();

    if (have_neon(cpu_flags))
        h264dsp_init_neon(c, bit_depth);
}
