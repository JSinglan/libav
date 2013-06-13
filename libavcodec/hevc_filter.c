#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "libavutil/pixdesc.h"
#include "libavutil/internal.h"
#include "cabac_functions.h"
#include "golomb.h"
#include "hevcdata.h"
#include "hevc.h"
#include "libavutil/opt.h"
#include "libavutil/md5.h"
#include "bit_depth_template.c"

#define LUMA 0
#define CB 1
#define CR 2

static int chroma_tc(HEVCContext *s, int qp_y, int c_idx)
{
    static int qp_c[] = { 29, 30, 31, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37 };
    int qp_i, offset;
    int qp;
    int idxt;

    // slice qp offset is not used for deblocking
    if (c_idx == 1)
        offset = s->pps->cb_qp_offset;
    else
        offset = s->pps->cr_qp_offset;

    qp_i = av_clip_c(qp_y + offset, - s->sps->qp_bd_offset, 57);
    if (qp_i < 30)
        qp = qp_i;
    else if (qp_i > 43)
        qp = qp_i - 6;
    else
        qp = qp_c[qp_i - 30];

    qp += s->sps->qp_bd_offset;

    idxt = av_clip_c(qp + DEFAULT_INTRA_TC_OFFSET + s->sh.tc_offset, 0, 53);
    return tctable[idxt];
}
static int get_qPy_pred(HEVCContext *s, int xC, int yC)
{
    int Log2CtbSizeY         = s->sps->log2_ctb_size;
    int Log2MinTrafoSize     = s->sps->log2_min_transform_block_size;
    int Log2MinCuQpDeltaSize = s->pps->log2_min_cu_qp_delta_size;
    int xQg                  = xC - ( xC & ( ( 1 << Log2MinCuQpDeltaSize) - 1 ) );
    int yQg                  = yC - ( yC & ( ( 1 << Log2MinCuQpDeltaSize) - 1 ) );
    int x                    = xC >> Log2MinCuQpDeltaSize;
    int y                    = yC >> Log2MinCuQpDeltaSize;
    int qPy_pred;
    int qPy_a;
    int qPy_b;
    int availableA           = ff_hevc_z_scan_block_avail(s, xC, yC, xQg-1, yQg  );
    int availableB           = ff_hevc_z_scan_block_avail(s, xC, yC, xQg  , yQg-1);
    int minTbAddrA;
    int minTbAddrB;
    int ctbAddrA = 0;
    int ctbAddrB = 0;
    if (availableA != 0) {
        int tmpX   = (xQg-1) >> Log2MinTrafoSize;
        int tmpY   =  yQg    >> Log2MinTrafoSize;
        minTbAddrA = s->pps->min_tb_addr_zs[ tmpX + tmpY * s->sps->pic_width_in_min_tbs];
        ctbAddrA   = ( minTbAddrA >> 2 ) * (Log2CtbSizeY - Log2MinTrafoSize);
    }
    if (availableB != 0) {
        int tmpX   =  xQg    >> Log2MinTrafoSize;
        int tmpY   = (yQg-1) >> Log2MinTrafoSize;
        minTbAddrB = s->pps->min_tb_addr_zs[ tmpX + tmpY * s->sps->pic_width_in_min_tbs];
        ctbAddrB   = ( minTbAddrB >> 2 ) * (Log2CtbSizeY - Log2MinTrafoSize);
    }
    // qPy_pred
    if (s->isFirstQPgroup != 0) {
        s->isFirstQPgroup = 0;
        qPy_pred = s->sh.slice_qp;
    } else {
        qPy_pred = s->qp_y;
    }
    // qPy_a
    if ( (availableA == 0) || (ctbAddrA != s->ctb_addr_ts) ) {
        qPy_a = qPy_pred;
    } else {
        qPy_a = s->qp_y_tab[(x-1) + y * s->sps->pic_width_in_min_tbs];
    }
    // qPy_b
    if ( (availableB == 0) || (ctbAddrB != s->ctb_addr_ts) ) {
        qPy_b = qPy_pred;
    } else {
        qPy_b = s->qp_y_tab[x + (y-1) * s->sps->pic_width_in_min_tbs];
    }
    return (qPy_a + qPy_b + 1) >> 1;
}
void ff_hevc_set_qPy(HEVCContext *s, int xC, int yC)
{
    int Log2CtbSizeY         = s->sps->log2_ctb_size;
    int Log2MinCuQpDeltaSize = s->pps->log2_min_cu_qp_delta_size;
    int x                    = xC >> Log2MinCuQpDeltaSize;
    int y                    = yC >> Log2MinCuQpDeltaSize;
    if (s->tu.cu_qp_delta != 0) {
        s->qp_y = ((get_qPy_pred(s, xC, yC) + s->tu.cu_qp_delta + 52 + 2 * s->sps->qp_bd_offset) %
                (52 + s->sps->qp_bd_offset)) - s->sps->qp_bd_offset;
    } else {
        s->qp_y = get_qPy_pred(s, xC, yC);
    }
    s->qp_y_tab[x + y * s->sps->pic_width_in_min_tbs] = s->qp_y;
}
static int get_qPy(HEVCContext *s, int xC, int yC)
{
    int Log2CtbSizeY         = s->sps->log2_ctb_size;
    int Log2MinCuQpDeltaSize = s->pps->log2_min_cu_qp_delta_size;
    int x                    = xC >> Log2MinCuQpDeltaSize;
    int y                    = yC >> Log2MinCuQpDeltaSize;
    return s->qp_y_tab[x + y * s->sps->pic_width_in_min_tbs];
}

static int get_pcm(HEVCContext *s, int x, int y)
{
    int log2_min_pu_size = s->sps->log2_min_pu_size - 1;
    int pic_width_in_min_pu = s->sps->pic_width_in_min_cbs * 4;
    return s->is_pcm[(y >> log2_min_pu_size) * pic_width_in_min_pu + (x >> log2_min_pu_size)];
}

#define TC_CALC(qp, bs) tctable[av_clip((qp) + DEFAULT_INTRA_TC_OFFSET * ((bs) - 1) + ((s->sh.tc_offset >> 1) << 1), 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET)]
void ff_hevc_deblocking_filter(HEVCContext *s)
{
    uint8_t *src;
    int chroma;
    uint8_t no_p[2] = {0};
    uint8_t no_q[2] = {0};
    int beta[2];
    int tc[2];
    int x, y;
    int c_tc[2];
    int pcmf = s->sps->pcm_enabled_flag && s->sps->pcm.loop_filter_disable_flag;

    // vertical filtering luma
    for (y = 0; y < s->sps->pic_height_in_luma_samples; y += 8) {
        for (x = 8; x < s->sps->pic_width_in_luma_samples; x += 8) {
            const int bs0 = s->vertical_bs[(x >> 3) + (y >> 2) * s->bs_width];
            const int bs1 = s->vertical_bs[(x >> 3) + ((y + 4) >> 2) * s->bs_width];
            if (bs0 || bs1) {
                const int qp0 = (get_qPy(s, x - 1, y) + get_qPy(s, x, y) + 1) >> 1;
                const int qp1 = (get_qPy(s, x - 1, y + 4) + get_qPy(s, x, y + 4) + 1) >> 1;
                beta[0] = betatable[av_clip(qp0 + ((s->sh.beta_offset >> 1) << 1), 0, MAX_QP)];
                beta[1] = betatable[av_clip(qp1 + ((s->sh.beta_offset >> 1) << 1), 0, MAX_QP)];
                tc[0] = bs0 ? TC_CALC(qp0, bs0) : 0;
                tc[1] = bs1 ? TC_CALC(qp1, bs1) : 0;

                if (pcmf) {
                        no_p[0] = get_pcm(s, x - 1, y);
                        no_p[1] = get_pcm(s, x - 1, y + 8);
                        no_q[0] = get_pcm(s, x, y);
                        no_q[1] = get_pcm(s, x, y + 8);
                }
                //const int no_p = pcmf ? get_pcm(s, x - 1, y) : 0;
                //const int no_q = pcmf ? get_pcm(s, x, y) : 0;

                src = &s->frame->data[LUMA][y * s->frame->linesize[LUMA] + x];
                s->hevcdsp.hevc_v_loop_filter_luma(src, s->frame->linesize[LUMA], beta, tc, no_p, no_q);
            }
        }
    }

    // vertical filtering CB and CR
    for (chroma = 1; chroma <= 2; chroma++) {
        for (y = 0; y < s->sps->pic_height_in_luma_samples; y += 16) {
            for (x = 16; x < s->sps->pic_width_in_luma_samples; x += 16) {
                const int bs0 = s->vertical_bs[(x >> 3) + (y >> 2) * s->bs_width];
                const int bs1 = s->vertical_bs[(x >> 3) + ((y + 8) >> 2) * s->bs_width];
                if ((bs0 == 2) || (bs1 == 2)) {
                    const int qp0 = (get_qPy(s, x - 1, y) + get_qPy(s, x, y) + 1) >> 1;
                    const int qp1 = (get_qPy(s, x - 1, y + 8) + get_qPy(s, x, y + 8) + 1) >> 1;
                    c_tc[0] = (bs0 == 2) ? chroma_tc(s, qp0, chroma) : 0;
                    c_tc[1] = (bs1 == 2) ? chroma_tc(s, qp1, chroma) : 0;
                    if (pcmf) {
                        no_p[0] = get_pcm(s, x - 1, y);
                        no_p[1] = get_pcm(s, x - 1, y + 8);
                        no_q[0] = get_pcm(s, x, y);
                        no_q[1] = get_pcm(s, x, y + 8);
                    }
                    src = &s->frame->data[chroma][(y / 2) * s->frame->linesize[chroma] + (x / 2)];
                    s->hevcdsp.hevc_v_loop_filter_chroma(src, s->frame->linesize[chroma], c_tc, no_p, no_q);
                }
            }
        }
    }

    // horizontal filtering luma
    for (y = 8; y < s->sps->pic_height_in_luma_samples; y += 8) {
        for (x = 0; x < s->sps->pic_width_in_luma_samples; x += 8) {
            const int bs0 = s->horizontal_bs[(x + y * s->bs_width) >> 2];
            const int bs1 = s->horizontal_bs[(x + 4 + y * s->bs_width) >> 2];
            if (bs0 || bs1) {
                const int qp0 = (get_qPy(s, x, y - 1) + get_qPy(s, x, y) + 1) >> 1;
                const int qp1 = (get_qPy(s, x + 4, y - 1) + get_qPy(s, x + 4, y) + 1) >> 1;
                beta[0]  = betatable[av_clip(qp0 + ((s->sh.beta_offset >> 1) << 1), 0, MAX_QP)];
                beta[1]  = betatable[av_clip(qp1 + ((s->sh.beta_offset >> 1) << 1), 0, MAX_QP)];
                tc[0] = bs0 ? TC_CALC(qp0, bs0) : 0;
                tc[1] = bs1 ? TC_CALC(qp1, bs1) : 0;
                if (pcmf) {
                        no_p[0] = get_pcm(s, x, y - 1);
                        no_p[1] = get_pcm(s, x + 8, y - 1);
                        no_q[0] = get_pcm(s, x, y);
                        no_q[1] = get_pcm(s, x + 8, y);
                }
                /*const int no_p = pcmf ? get_pcm(s, x, y - 1) : 0;
                const int no_q = pcmf ? get_pcm(s, x, y) : 0;*/
                src = &s->frame->data[LUMA][y * s->frame->linesize[LUMA] + x];
                s->hevcdsp.hevc_h_loop_filter_luma(src, s->frame->linesize[LUMA], beta, tc, no_p, no_q);
            }
        }
    }
    // horizontal filtering CB and CR
    for (chroma = 1; chroma <= 2; chroma++) {
        for (y = 16; y < s->sps->pic_height_in_luma_samples; y += 16) {
            for (x = 0; x < s->sps->pic_width_in_luma_samples; x += 16) {
                const int bs0 = s->horizontal_bs[(x + y * s->bs_width) >> 2];
                const int bs1 = s->horizontal_bs[(x + 8 + y * s->bs_width) >> 2];
                if ((bs0 == 2) || (bs1 == 2)) {
                    const int qp0 = (get_qPy(s, x, y - 1) + get_qPy(s, x, y) + 1) >> 1;
                    const int qp1 = (get_qPy(s, x + 8, y - 1) + get_qPy(s, x + 8, y) + 1) >> 1;
                    c_tc[0] = (bs0 == 2) ? chroma_tc(s, qp0, chroma) : 0;
                    c_tc[1] = (bs1 == 2) ? chroma_tc(s, qp1, chroma) : 0;
                    if (pcmf) {
                        no_p[0] = get_pcm(s, x, y - 1);
                        no_p[1] = get_pcm(s, x + 8, y - 1);
                        no_q[0] = get_pcm(s, x, y);
                        no_q[1] = get_pcm(s, x + 8, y);
                    }
                    src = &s->frame->data[chroma][(y / 2) * s->frame->linesize[chroma] + (x / 2)];
                    s->hevcdsp.hevc_h_loop_filter_chroma(src, s->frame->linesize[chroma], c_tc, no_p, no_q);
                }
            }
        }
    }
}
#undef TC_CALC

#define CTB(tab, x, y) ((tab)[(y) * s->sps->pic_width_in_ctbs + (x)])
#ifndef SAO_IN_LOOP
void ff_hevc_sao_filter(HEVCContext *s)
{
    //TODO: This should be easily parallelizable
    //TODO: skip CBs when (cu_transquant_bypass_flag || (pcm_loop_filter_disable_flag && pcm_flag))
    int c_idx, y_ctb, x_ctb;
    int c_idx_min = s->sh.slice_sample_adaptive_offset_flag[0] != 0 ? 0 : 1;
    int c_idx_max = s->sh.slice_sample_adaptive_offset_flag[1] != 0 ? 3 : 1;
    for (c_idx = c_idx_min; c_idx < c_idx_max; c_idx++) {
        int stride = s->frame->linesize[c_idx];
        int ctb_size = (1 << (s->sps->log2_ctb_size)) >> s->sps->hshift[c_idx];
        for (y_ctb = 0; y_ctb < s->sps->pic_height_in_ctbs; y_ctb++) {
            for (x_ctb = 0; x_ctb < s->sps->pic_width_in_ctbs; x_ctb++) {
                struct SAOParams *sao = &CTB(s->sao, x_ctb, y_ctb);
                int x = x_ctb * ctb_size;
                int y = y_ctb * ctb_size;
                int width = FFMIN(ctb_size,
                                  (s->sps->pic_width_in_luma_samples >> s->sps->hshift[c_idx]) - x);
                int height = FFMIN(ctb_size,
                                   (s->sps->pic_height_in_luma_samples >> s->sps->vshift[c_idx]) - y);
#ifdef DEBLOCKING_IN_LOOP
                uint8_t *src = &s->dbf_frame->data[c_idx][y * stride + x];
#else
                uint8_t *src = &s->frame->data[c_idx][y * stride + x];
#endif
                uint8_t *dst = &s->sao_frame->data[c_idx][y * stride + x];
                switch (sao->type_idx[c_idx]) {
                case SAO_BAND:
                    s->hevcdsp.sao_band_filter(dst, src, stride, sao->offset_val[c_idx],
                                               sao->band_position[c_idx], width, height);
                    break;
                case SAO_EDGE: {
                    int top    = y_ctb == 0;
                    int bottom = y_ctb == (s->sps->pic_height_in_ctbs - 1);
                    int left   = x_ctb == 0;
                    int right  = x_ctb == (s->sps->pic_width_in_ctbs - 1);
                    s->hevcdsp.sao_edge_filter(dst, src, stride, sao->offset_val[c_idx],
                                               sao->eo_class[c_idx],
                                               top, bottom, left, right, width, height);
                    break;
                }
                }
            }
        }
    }
}
#else
void ff_hevc_sao_filter(HEVCContext *s, int x0, int y0)
{
    //TODO: This should be easily parallelizable
    //TODO: skip CBs when (cu_transquant_bypass_flag || (pcm_loop_filter_disable_flag && pcm_flag))
    int c_idx;
    int c_idx_min = s->sh.slice_sample_adaptive_offset_flag[0] != 0 ? 0 : 1;
    int c_idx_max = s->sh.slice_sample_adaptive_offset_flag[1] != 0 ? 3 : 1;
    int x_ctb = x0 >> s->sps->log2_ctb_size;
    int y_ctb = y0 >> s->sps->log2_ctb_size;
    int x, y, x_end, y_end;
    x_end = x0+(1<<s->sps->log2_ctb_size);
    if (x_end > s->sps->pic_width_in_luma_samples)
        x_end = s->sps->pic_width_in_luma_samples;
    y_end = y0+(1<<s->sps->log2_ctb_size);
    if (y_end > s->sps->pic_height_in_luma_samples)
        y_end = s->sps->pic_height_in_luma_samples;

    for (y = y0; y < y_end; y++) {
        for (x = x0; x < x_end; x ++) {
            s->sao_frame->data[LUMA][y * s->sao_frame->linesize[LUMA] + x] = s->frame->data[LUMA][y * s->frame->linesize[LUMA] + x];
        }
    }
    for (y = y0/2; y < y_end/2; y++) {
        for (x = x0/2; x < x_end/2; x ++) {
            s->sao_frame->data[CB][y * s->sao_frame->linesize[CB] + x] = s->frame->data[CB][y * s->frame->linesize[CB] + x];
            s->sao_frame->data[CR][y * s->sao_frame->linesize[CR] + x] = s->frame->data[CR][y * s->frame->linesize[CR] + x];
        }
    }

    printf("ff_hevc_sao_filter(s, %d, %d) : (%d, %d)\n", x0, y0, x_ctb, y_ctb);
    for (c_idx = c_idx_min; c_idx < c_idx_max; c_idx++) {
        int stride = s->frame->linesize[c_idx];
        int ctb_size = (1 << (s->sps->log2_ctb_size)) >> s->sps->hshift[c_idx];
        struct SAOParams *sao = &CTB(s->sao, x_ctb, y_ctb);
        int x = x_ctb * ctb_size;
        int y = y_ctb * ctb_size;

        int width = FFMIN(ctb_size,
                                  (s->sps->pic_width_in_luma_samples >> s->sps->hshift[c_idx]) - x);
        int height = FFMIN(ctb_size,
                                    (s->sps->pic_height_in_luma_samples >> s->sps->vshift[c_idx]) - y);
        if (c_idx==0)
        printf("sao(%d -> %d, %d -> %d)\n",x,x+width, y, y+height);
        uint8_t *src = &s->frame->data[c_idx][y * stride + x];
        uint8_t *dst = &s->sao_frame->data[c_idx][y * stride + x];
        switch (sao->type_idx[c_idx]) {
            case SAO_BAND:
                s->hevcdsp.sao_band_filter(dst, src, stride, sao->offset_val[c_idx],
                        sao->band_position[c_idx], width, height);
                break;
            case SAO_EDGE: {
                int top    = y_ctb == 0;
                int bottom = y_ctb == (s->sps->pic_height_in_ctbs - 1);
                int left   = x_ctb == 0;
                int right  = x_ctb == (s->sps->pic_width_in_ctbs - 1);
                s->hevcdsp.sao_edge_filter(dst, src, stride, sao->offset_val[c_idx],
                                        sao->eo_class[c_idx],
                                        top, bottom, left, right, width, height);
                break;
            }
        }
    }
}
#endif
#undef CTB

static int boundary_strength(HEVCContext *s, MvField *curr, uint8_t curr_cbf_luma, MvField *neigh, uint8_t neigh_cbf_luma, int tu_border)
{
    if (tu_border) {
        if (curr->is_intra || neigh->is_intra)
            return 2;
        if (curr_cbf_luma || neigh_cbf_luma)
            return 1;
    }

    if (s->sh.slice_type == P_SLICE) {
        if (abs(neigh->mv[0].x - curr->mv[0].x) >= 4 || abs(neigh->mv[0].y - curr->mv[0].y) >= 4 ||
            s->ref->refPicList[0].list[neigh->ref_idx[0]] != s->ref->refPicList[0].list[curr->ref_idx[0]])
            return 1;
        else
            return 0;
    } else if (s->sh.slice_type == B_SLICE) {
        int mvs = curr->pred_flag[0] + curr->pred_flag[1];
        if (mvs == neigh->pred_flag[0] + neigh->pred_flag[1]) {
            if (mvs == 2) {
                // same L0 and L1
                if (s->ref->refPicList[0].list[curr->ref_idx[0]] == s->ref->refPicList[0].list[neigh->ref_idx[0]]
                    && s->ref->refPicList[0].list[curr->ref_idx[0]] == s->ref->refPicList[1].list[curr->ref_idx[1]]
                    && s->ref->refPicList[0].list[neigh->ref_idx[0]] == s->ref->refPicList[1].list[neigh->ref_idx[1]]) {
                    if ((abs(neigh->mv[0].x - curr->mv[0].x) >= 4 || abs(neigh->mv[0].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[1].x - curr->mv[1].x) >= 4 || abs(neigh->mv[1].y - curr->mv[1].y) >= 4) &&
                        (abs(neigh->mv[1].x - curr->mv[0].x) >= 4 || abs(neigh->mv[1].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[0].x - curr->mv[1].x) >= 4 || abs(neigh->mv[0].y - curr->mv[1].y) >= 4))
                        return 1;
                    else
                        return 0;
                }
                else if (s->ref->refPicList[0].list[neigh->ref_idx[0]] == s->ref->refPicList[0].list[curr->ref_idx[0]]
                         && s->ref->refPicList[1].list[neigh->ref_idx[1]] == s->ref->refPicList[1].list[curr->ref_idx[1]]) {
                    if (abs(neigh->mv[0].x - curr->mv[0].x) >= 4 || abs(neigh->mv[0].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[1].x - curr->mv[1].x) >= 4 || abs(neigh->mv[1].y - curr->mv[1].y) >= 4)
                        return 1;
                    else
                        return 0;
                }
                else if (s->ref->refPicList[1].list[neigh->ref_idx[1]] == s->ref->refPicList[0].list[curr->ref_idx[0]]
                        && s->ref->refPicList[0].list[neigh->ref_idx[0]] == s->ref->refPicList[1].list[curr->ref_idx[1]]) {
                    if (abs(neigh->mv[1].x - curr->mv[0].x) >= 4 || abs(neigh->mv[1].y - curr->mv[0].y) >= 4 ||
                        abs(neigh->mv[0].x - curr->mv[1].x) >= 4 || abs(neigh->mv[0].y - curr->mv[1].y) >= 4)
                        return 1;
                    else
                        return 0;
                } else {
                    return 1;
                }
            } else { // 1 MV
                Mv A, B;
                int ref_A;
                int ref_B;
                if (curr->pred_flag[0]) {
                    A = curr->mv[0];
                    ref_A = s->ref->refPicList[0].list[curr->ref_idx[0]];
                }
                else {
                    A = curr->mv[1];
                    ref_A = s->ref->refPicList[1].list[curr->ref_idx[1]];
                }
                if (neigh->pred_flag[0]) {
                    B = neigh->mv[0];
                    ref_B = s->ref->refPicList[0].list[neigh->ref_idx[0]];
                } else {
                    B = neigh->mv[1];
                    ref_B = s->ref->refPicList[1].list[neigh->ref_idx[1]];
                }
                if (ref_A == ref_B) {
                    if (abs(A.x - B.x) >= 4 || abs(A.y - B.y) >= 4)
                        return 1;
                    else
                        return 0;
                } else
                    return 1;
            }
        }
        else
            return 1;
    }
    return 0;
}
void ff_hevc_deblocking_boundary_strengths(HEVCContext *s, int x0, int y0, int log2_trafo_size)
{
    int log2_min_pu_size = s->sps->log2_min_pu_size;
    int min_pu_size = 1 << s->sps->log2_min_pu_size;
    int pic_width_in_min_pu = s->sps->pic_width_in_min_cbs * 4;
    int i, j;
    int bs;
    MvField *tab_mvf = s->ref->tab_mvf;
    if ((y0 & 7) == 0) {
        int yp_pu = (y0 - 1) / min_pu_size;
        int yq_pu = y0 >> log2_min_pu_size;
        for (i = 0; i < (1<<log2_trafo_size); i+=4) {
            int x_pu = (x0 + i) >> log2_min_pu_size;
            MvField *top = &tab_mvf[yp_pu * pic_width_in_min_pu + x_pu];
            MvField *curr = &tab_mvf[yq_pu * pic_width_in_min_pu + x_pu];
            uint8_t top_cbf_luma = s->cbf_luma[yp_pu * pic_width_in_min_pu + x_pu];
            uint8_t curr_cbf_luma = s->cbf_luma[yq_pu * pic_width_in_min_pu + x_pu];
            bs = boundary_strength(s, curr, curr_cbf_luma, top, top_cbf_luma, 1);
            if (bs)
                s->horizontal_bs[((x0 + i) + y0 * s->bs_width) >> 2] = bs;
        }
    }
    // bs for TU internal horizontal PU boundaries
    if (log2_trafo_size > s->sps->log2_min_pu_size && s->sh.slice_type != I_SLICE)
        for (j = 8; j < (1<<log2_trafo_size); j += 8) {
            int yp_pu = (y0 + j - 1) >> log2_min_pu_size;
            int yq_pu = (y0 + j) >> log2_min_pu_size;
            for (i = 0; i < (1<<log2_trafo_size); i += 4) {
                int x_pu = (x0 + i) >> log2_min_pu_size;
                MvField *top = &tab_mvf[yp_pu * pic_width_in_min_pu + x_pu];
                MvField *curr = &tab_mvf[yq_pu * pic_width_in_min_pu + x_pu];
                uint8_t top_cbf_luma = s->cbf_luma[yp_pu * pic_width_in_min_pu + x_pu];
                uint8_t curr_cbf_luma = s->cbf_luma[yq_pu * pic_width_in_min_pu + x_pu];
                bs = boundary_strength(s, curr, curr_cbf_luma, top, top_cbf_luma, 0);
                if (bs)
                    s->horizontal_bs[((x0 + i) + (y0 + j) * s->bs_width) >> 2] = bs;
            }
        }
    // bs for vertical TU boundaries
    if ((x0 & 7) == 0) {
        int xp_pu = (x0 - 1) / min_pu_size;
        int xq_pu = x0 >> log2_min_pu_size;
        for (i = 0; i < (1<<log2_trafo_size); i+=4) {
            int y_pu = (y0 + i) >> log2_min_pu_size;
            MvField *left = &tab_mvf[y_pu * pic_width_in_min_pu + xp_pu];
            MvField *curr = &tab_mvf[y_pu * pic_width_in_min_pu + xq_pu];
            uint8_t left_cbf_luma = s->cbf_luma[y_pu * pic_width_in_min_pu + xp_pu];
            uint8_t curr_cbf_luma = s->cbf_luma[y_pu * pic_width_in_min_pu + xq_pu];
            bs = boundary_strength(s, curr, curr_cbf_luma, left, left_cbf_luma, 1);
            if (bs)
                s->vertical_bs[(x0 >> 3) + ((y0 + i) >> 2) * s->bs_width] = bs;
        }
    }
    // bs for TU internal vertical PU boundaries
    if (log2_trafo_size > s->sps->log2_min_pu_size && s->sh.slice_type != I_SLICE)
        for (j = 0; j < (1<<log2_trafo_size); j += 4) {
            int y_pu = (y0 + j) >> log2_min_pu_size;
            for (i = 8; i < (1<<log2_trafo_size); i += 8) {
                int xp_pu = (x0 + i - 1) >> log2_min_pu_size;
                int xq_pu = (x0 + i) >> log2_min_pu_size;
                MvField *left = &tab_mvf[y_pu * pic_width_in_min_pu + xp_pu];
                MvField *curr = &tab_mvf[y_pu * pic_width_in_min_pu + xq_pu];
                uint8_t left_cbf_luma = s->cbf_luma[y_pu * pic_width_in_min_pu + xp_pu];
                uint8_t curr_cbf_luma = s->cbf_luma[y_pu * pic_width_in_min_pu + xq_pu];
                bs = boundary_strength(s, curr, curr_cbf_luma, left, left_cbf_luma, 0);
                if (bs)
                    s->vertical_bs[((x0 + i) >> 3) + ((y0 + j) >> 2) * s->bs_width] = bs;
            }
        }
}
#undef LUMA
#undef CB
#undef CR
