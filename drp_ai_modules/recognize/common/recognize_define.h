/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2023 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name    : recognize_define.h
 * Version      : 7.30
 * Description  : RZ/V DRP-AI Sample Application for USB Camera version
 ***********************************************************************************************************************/

#ifndef RECOGNIE_DEFINE_H
#define RECOGNIE_DEFINE_H

#define BOARD_IP "192.168.1.11"
#define DRP_MAX_FREQ (-1)
#define DRPAI_FREQ (-1)

#define JPEG_QUALUTY (50)    /* JPEG 0 - 100 */
#define MODEL_RESOLUTION (0) /* 0=VGA(640x480) 1=HD(1280x720) 2=FHD(1920x1080) */

#define SEQUENCTCIAL             /* Enable sync capture and inference*/
#define INFERENE_THREAD_ON       /* Enable Inference Thread */
#define DRPAI_INFERENCE_ON       /* Enable DRP-AI inferenece */
#define SEND_INFERENCE_RESULT_ON /* Enable send inference image*/
//#define SEND_CAMERA_THROUGH_ON                      /* Enable send through image */

//#define COUT_INFERENCE_RESULT_ON                    /* Enable print inference result to console */
//#define COUT_INFERENCE_MSG_ON                       /* Enable print inference thread information */
//#define COUT_CAPTURE_MSG_ON                         /* Enable print capture thread information */
//#define PRINT_MEASURE_ON                            /* Enable print measure time */

//#define DRPAI_REG_DUMP_ON                           /* Enable DRP-AI Register and desc dump */
#endif // !RECOGNIE_DEFINE_H
