/*
 * linux/drivers/media/video/codadx6/codadx6_regs.h
 *
 * Copyright (C) 2012 Vista Silicon SL
 *    Javier Martin <javier.martin@vista-silicon.com>
 *    Xavier Duret
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _REGS_CODADX6_H_
#define _REGS_CODADX6_H_

/* HW registers */
#define CODADX6_REG_BIT_CODE_RUN		0x000
#define		CODADX6_REG_RUN_ENABLE		(1 << 0)
#define CODADX6_REG_BIT_CODE_DOWN		0x004
/* Internal SRAM short address in the BIT */
#define 	CODADX6_DOWN_ADDRESS_SET(x)	(((x) & 0xffff) << 16)
#define		CODADX6_DOWN_DATA_SET(x)	((x) & 0xffff)
#define CODADX6_REG_BIT_HOST_IN_REQ		0x008
#define CODADX6_REG_BIT_INT_CLEAR		0x00C
#define		CODADX6_REG_BIT_INT_CLEAR_SET	0x1
#define CODADX6_REG_BIT_INT_STATUS		0x010
#define CODADX6_REG_BIT_CODE_RESET		0x014
#define		CODADX6_REG_RESET_ENABLE	(1 << 0)
#define CODADX6_REG_BIT_CUR_PC			0x018

/* Static SW registers */
#define CODADX6_REG_BIT_CODE_BUF_ADDR		0x100
#define CODADX6_REG_BIT_WORK_BUF_ADDR		0x104
#define CODADX6_REG_BIT_PARA_BUF_ADDR		0x108
#define CODADX6_REG_BIT_STREAM_CTRL		0x10C
#define		CODADX6_STREAM_UNDOCUMENTED	(1 << 2)
		/* Stream Full Empty Check Disable */
#define 	CODADX6_STREAM_CHKDIS_OFFSET	(1 << 1)
		/* Stream Endianess */
#define		CODADX6_STREAM_ENDIAN_SELECT	(1 << 0)
#define CODADX6_REG_BIT_FRAME_MEM_CTRL		0x110
		/* Image Endianess */
#define 	CODADX6_IMAGE_ENDIAN_SELECT	(1 << 0)
#define CODADX6_REG_BIT_RD_PTR_0              0x120
#define CODADX6_REG_BIT_WR_PTR_0              0x124
#define CODADX6_REG_BIT_SEARCH_RAM_BASE_ADDR  0x140
#define CODADX6_REG_BIT_BUSY			0x160
#define 	CODADX6_REG_BIT_BUSY_FLAG	1
#define CODADX6_REG_BIT_RUN_COMMAND           0x164
#define 	CODADX6_COMMAND_SEQ_INIT                         1
#define 	CODADX6_COMMAND_SEQ_END                          2
#define 	CODADX6_COMMAND_PIC_RUN                          3
#define 	CODADX6_COMMAND_SET_FRAME_BUF                    4
#define 	CODADX6_COMMAND_ENCODE_HEADER                    5
#define 	CODADX6_COMMAND_ENC_PARA_SET                     6
#define 	CODADX6_COMMAND_DEC_PARA_SET                     7
#define 	CODADX6_COMMAND_DEC_BUF_FLUSH                    8
#define 	CODADX6_COMMAND_RC_CHANGE_PARAMETER              9
#define 	CODADX6_COMMAND_FIRMWARE_GET                     0xf
#define CODADX6_REG_BIT_RUN_INDEX		0x168
#define		CODADX6_INDEX_SET(x)		((x) & 0x3)
#define CODADX6_REG_BIT_RUN_COD_STD		0x16C
#define 	CODADX6_MODE_DECODE_M4S2	0
#define 	CODADX6_MODE_ENCODE_M4S2	1
#define 	CODADX6_MODE_DECODE_H264	2
#define 	CODADX6_MODE_ENCODE_H264	3
#define 	CODADX6_MODE_DECODE_WVC1	4
#define 	CODADX6_MODE_INVALID		0xffff
#define CODADX6_REG_BIT_INT_ENABLE		0x170
#define		CODADX6_INT_INTERRUPT_ENABLE	(1 << 3)

/*
 * Commands' mailbox:
 * registers with offsets in the range 0x180-0x1D0
 * have different meaning depending on the command being
 * issued.
 */
/* Encoder Sequence Initialization */
#define CODADX6_CMD_ENC_SEQ_BB_START          0x180
#define CODADX6_CMD_ENC_SEQ_BB_SIZE           0x184
#define CODADX6_CMD_ENC_SEQ_OPTION            0x188
#define 	CODADX6_OPTION_GAMMA_OFFSET                      7
#define 	CODADX6_OPTION_GAMMA_MASK                        0x01
#define 	CODADX6_OPTION_LIMITQP_OFFSET                    6
#define 	CODADX6_OPTION_LIMITQP_MASK                      0x01
#define 	CODADX6_OPTION_RCINTRAQP_OFFSET                  5
#define 	CODADX6_OPTION_RCINTRAQP_MASK                    0x01
#define 	CODADX6_OPTION_FMO_OFFSET                        4
#define 	CODADX6_OPTION_FMO_MASK                          0x01
// /* There is no bit 3 */
// #define 	CODADX6_OPTION_AUD_OFFSET                        2
// #define 	CODADX6_OPTION_AUD_MASK                          0x01
#define 	CODADX6_OPTION_SLICEREPORT_OFFSET                1
#define 	CODADX6_OPTION_SLICEREPORT_MASK                  0x01
// /* There is no bit 0 */
#define CODADX6_CMD_ENC_SEQ_COD_STD           0x18C
#define 	CODADX6_ENCODE_MPEG4                             0
#define 	CODADX6_ENCODE_H263                              1
#define 	CODADX6_ENCODE_H264                              2
#define CODADX6_CMD_ENC_SEQ_SRC_SIZE          0x190
#define 	CODADX6_PICWIDTH_OFFSET                          10
#define 	CODADX6_PICWIDTH_MASK                            0x3ff
#define 	CODADX6_PICHEIGHT_OFFSET                         0
#define 	CODADX6_PICHEIGHT_MASK                           0x3ff
#define CODADX6_CMD_ENC_SEQ_SRC_F_RATE        0x194
#define CODADX6_CMD_ENC_SEQ_MP4_PARA          0x198
#define 	CODADX6_MP4PARAM_VERID_OFFSET                    6
#define 	CODADX6_MP4PARAM_VERID_MASK                      0x01
/* intra_dc_vlc_thr in MPEG-4 part 2 standard: unsigned [0:7] */
#define 	CODADX6_MP4PARAM_INTRADCVLCTHR_OFFSET            2
#define 	CODADX6_MP4PARAM_INTRADCVLCTHR_MASK              0x07
#define 	CODADX6_MP4PARAM_REVERSIBLEVLCENABLE_OFFSET      1
#define 	CODADX6_MP4PARAM_REVERSIBLEVLCENABLE_MASK        0x01
#define 	CODADX6_MP4PARAM_DATAPARTITIONENABLE_OFFSET      0
#define 	CODADX6_MP4PARAM_DATAPARTITIONENABLE_MASK        0x01
// #define CODADX6_CMD_ENC_SEQ_263_PARA          0x19C
// #define 	CODADX6_263PARAM_ANNEXJENABLE_OFFSET             2
// #define 	CODADX6_263PARAM_ANNEXJENABLE_MASK               0x01
// #define 	CODADX6_263PARAM_ANNEXKENABLE_OFFSET             1
// #define 	CODADX6_263PARAM_ANNEXKENABLE_MASK               0x01
// #define 	CODADX6_263PARAM_ANNEXTENABLE_OFFSET             0
// #define 	CODADX6_263PARAM_ANNEXTENABLE_MASK               0x01
#define CODADX6_CMD_ENC_SEQ_264_PARA          0x1A0
/* deblk_filter_offset_alpha: signed [-6:6] */
#define 	CODADX6_264PARAM_DEBLKFILTEROFFSETBETA_OFFSET    12
#define 	CODADX6_264PARAM_DEBLKFILTEROFFSETBETA_MASK      0x0f
/* deblk_filter_offset_beta: signed [-6:6] */
#define 	CODADX6_264PARAM_DEBLKFILTEROFFSETALPHA_OFFSET   8
#define 	CODADX6_264PARAM_DEBLKFILTEROFFSETALPHA_MASK     0x0f
#define 	CODADX6_264PARAM_DISABLEDEBLK_OFFSET             6
#define 	CODADX6_264PARAM_DISABLEDEBLK_MASK               0x01
#define 	CODADX6_264PARAM_CONSTRAINEDINTRAPREDFLAG_OFFSET 5
#define 	CODADX6_264PARAM_CONSTRAINEDINTRAPREDFLAG_MASK   0x01
/* chroma_qp_offset: signed [-12:12] */
#define 	CODADX6_264PARAM_CHROMAQPOFFSET_OFFSET           0
#define 	CODADX6_264PARAM_CHROMAQPOFFSET_MASK             0x1f
#define CODADX6_CMD_ENC_SEQ_SLICE_MODE        0x1A4
/* Slice size */
#define 	CODADX6_SLICING_SIZE_OFFSET                      2
#define 	CODADX6_SLICING_SIZE_MASK                        0x3fffffff
/* Unit used for slice size: 0 = bits per slice, 1 = Macroblocks per slice */
#define 	CODADX6_SLICING_UNIT_OFFSET                      1
#define 	CODADX6_SLICING_UNIT_MASK                        0x01
/* Slicing mode: 0 = One slice per picture, 1 = Multiple slices per picture */
#define 	CODADX6_SLICING_MODE_OFFSET                      0
#define 	CODADX6_SLICING_MODE_MASK                        0x01
#define CODADX6_CMD_ENC_SEQ_GOP_SIZE          0x1A8
/* GOP Size: 0 = Only first picture is Intra, 1 = All pictures are Intra
             n from 2 to 60 = One picture out of n is Intra */
#define 	CODADX6_GOP_SIZE_OFFSET                          0
#define 	CODADX6_GOP_SIZE_MASK                            0x3f
#define CODADX6_CMD_ENC_SEQ_RC_PARA           0x1AC
/* Disable autoskip: 1 = Do not skip a frame if bitstream is bigger than specified */
#define 	CODADX6_RATECONTROL_AUTOSKIP_OFFSET              31
#define 	CODADX6_RATECONTROL_AUTOSKIP_MASK                0x01
/* Initial delay: time in ms to fill the VBV buffer */
#define 	CODADX6_RATECONTROL_INITIALDELAY_OFFSET          16
#define 	CODADX6_RATECONTROL_INITIALDELAY_MASK            0x7f
/* Bitrate: in kilobits per seconds */
#define 	CODADX6_RATECONTROL_BITRATE_OFFSET               1
#define 	CODADX6_RATECONTROL_BITRATE_MASK                 0x7f
#define 	CODADX6_RATECONTROL_ENABLE_OFFSET                0
#define 	CODADX6_RATECONTROL_ENABLE_MASK                  0x01
#define CODADX6_CMD_ENC_SEQ_RC_BUF_SIZE       0x1B0
#define CODADX6_CMD_ENC_SEQ_INTRA_REFRESH     0x1B4
#define CODADX6_CMD_ENC_SEQ_FMO               0x1B8
/* Flexible Macroblock Ordering type: 0 = interleaved, 1 = dispersed */
#define 	CODADX6_FMOPARAM_TYPE_OFFSET                     4
#define 	CODADX6_FMOPARAM_TYPE_MASK                       1
/* Flexible Macroblock Ordering Slice Number: unsigned [2:8] */
#define 	CODADX6_FMOPARAM_SLICENUM_OFFSET                 0
#define 	CODADX6_FMOPARAM_SLICENUM_MASK                   0x0f
// #define CODADX6_CMD_ENC_SEQ_INTRA_QP          0x1BC
#define CODADX6_CMD_ENC_SEQ_RC_QP_MAX         0x1C8
/* QP: from 1 to 51 in H.264 */
#define 	CODADX6_QPMAX_OFFSET                             0
#define 	CODADX6_QPMAX_MASK                               0x3f
#define CODADX6_CMD_ENC_SEQ_RC_GAMMA          0x1CC
#define 	CODADX6_GAMMA_OFFSET                             0
#define 	CODADX6_GAMMA_MASK                               0xffff
#define CODADX6_RET_ENC_SEQ_SUCCESS           0x1C0

// /* Encoder Picture Run */
#define CODADX6_CMD_ENC_PIC_SRC_ADDR_Y        0x180
#define CODADX6_CMD_ENC_PIC_SRC_ADDR_CB       0x184
#define CODADX6_CMD_ENC_PIC_SRC_ADDR_CR       0x188
#define CODADX6_CMD_ENC_PIC_QS                0x18C
#define CODADX6_CMD_ENC_PIC_ROT_MODE          0x190
#define CODADX6_CMD_ENC_PIC_OPTION            0x194
#define CODADX6_CMD_ENC_PIC_BB_START          0x198
#define CODADX6_CMD_ENC_PIC_BB_SIZE           0x19C
#define CODADX6_RET_ENC_PIC_TYPE              0x1C4
#define CODADX6_RET_ENC_PIC_SLICE_NUM         0x1CC
#define CODADX6_RET_ENC_PIC_FLAG              0x1D0

/* Set Frame Buffer */
#define CODADX6_CMD_SET_FRAME_BUF_NUM         0x180
#define CODADX6_CMD_SET_FRAME_BUF_STRIDE      0x184

/* Encoder Header */
#define CODADX6_CMD_ENC_HEADER_CODE           0x180
#define 	CODADX6_GAMMA_OFFSET                             0
#define 	CODADX6_HEADER_H264_SPS                          0
#define 	CODADX6_HEADER_H264_PPS                          1
#define 	CODADX6_HEADER_MP4V_VOL                          0
#define 	CODADX6_HEADER_MP4V_VOS                          1
#define 	CODADX6_HEADER_MP4V_VIS                          2
#define CODADX6_CMD_ENC_HEADER_BB_START       0x184
#define CODADX6_CMD_ENC_HEADER_BB_SIZE        0x188

// /* Set Encoder Parameter */
// #define CODADX6_CMD_ENC_PARA_SET_TYPE         0x180
// #define CODADX6_RET_ENC_PARA_SET_SIZE         0x1c0
// 
/* Get Version */
#define CODADX6_CMD_FIRMWARE_VERNUM		0x1c0
#define		CODADX6_FIRMWARE_PRODUCT(x)	(((x) >> 16) & 0xffff)
#define		CODADX6_FIRMWARE_MAJOR(x)	(((x) >> 12) & 0x0f)
#define		CODADX6_FIRMWARE_MINOR(x)	(((x) >> 8) & 0x0f)
#define		CODADX6_FIRMWARE_RELEASE(x)	((x) & 0xff)

#endif
