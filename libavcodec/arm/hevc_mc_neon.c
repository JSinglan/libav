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


#include <arm_neon.h>

#include "config.h"
#include "libavutil/avassert.h"
#include "libavutil/pixdesc.h"
#include "libavcodec/get_bits.h"
#include "libavcodec/hevcdata.h"
#include "libavcodec/hevc.h"

#define BIT_DEPTH 8

void ff_hevc_put_unweighted_pred_8_arm(uint8_t *_dst, ptrdiff_t _dststride,
        int16_t *src, ptrdiff_t srcstride, int width, int height);

void ff_hevc_put_unweighted_pred_8_arm(uint8_t *_dst, ptrdiff_t _dststride,
        int16_t *src, ptrdiff_t srcstride, int width, int height) {
    int x, y;
    uint8_t *dst = (uint8_t*) _dst;
    ptrdiff_t dststride = _dststride / sizeof(uint8_t);

    int16x8_t r0, r1, f0;
    uint8x8_t o0, o1;
    uint8x16_t out;
#if BIT_DEPTH < 14
    int16_t offset = 1 << (13 - BIT_DEPTH);
#else
    int16_t offset = 0;
#endif
    f0 = vld1q_dup_s16(&offset);

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x += 16) {
            r0 = vld1q_s16(&src[x]);
            r1 = vld1q_s16(&src[x + 8]);
            r0 = vqaddq_s16(r0, f0);

            r1 = vqaddq_s16(r1, f0);
            r0 = vshrq_n_s16(r0, 14 - BIT_DEPTH);
            r1 = vshrq_n_s16(r1, 14 - BIT_DEPTH);
            o0 = vqmovun_s16(r0);
            o1 = vqmovun_s16(r1);
            out = vcombine_u8(o0, o1);
            vst1q_u8(&dst[x], out);
        }
        dst += dststride;
        src += srcstride;
    }
}

