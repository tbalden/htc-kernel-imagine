/*****************************************************************************
* File: track-report.h
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*
*
*****************************************************************************/
#ifndef TRACKREPORT_H
#define TRACKREPORT_H

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/


// Piezo bar Track Report

struct __attribute__((__packed__)) track_report {
    uint8_t  trk_id : 5;    // track_id
    uint8_t  bar_id : 3;    // bar_id
    uint8_t  force_lvl;     // force of touch
    uint16_t  center;       // center position, pos0
    uint16_t  bottom;       // bottom position, pos1
    uint16_t  top;          // top position, pos2
};

// Strain Gauge Track Report

#define STG_ELMT_MAX                7

// Strain Gauge bar_id = {6, 7}
#define STG_START_BAR_ID            6
#define IS_STG_TRACK_REPORT(barid)  ((int)((barid)>= STG_START_BAR_ID))

struct __attribute__((__packed__)) stg_track_report {
    uint8_t  rsvd   : 5;
    uint8_t  bar_id : 3;                    // bar_id
    uint8_t  force_lvl[STG_ELMT_MAX];       // force of touch for i'th gauge
};


#define TR_DIAG_MAX_MPATH                 10    // num in PH_0006_LP_Pixie

typedef struct tr_diag_vers_001_s
{
    uint8_t           start_code;       // always 0x00
    uint8_t           vers;             // 0x01 in this case
    uint16_t          mpa[TR_DIAG_MAX_MPATH];     // D1
    uint8_t           d_mp[TR_DIAG_MAX_MPATH];    // D1
    uint8_t           ntm;                        // D1
    uint8_t           gpio;                       // C2
    uint8_t           atc;                        // C2
    uint8_t           rsvd[5];

} tr_diag_vers_001_t, *p_tr_diag_vers_001_t;

typedef struct tr_diag_vers_002_s
{
    uint8_t           start_code;       // always 0x00
    uint8_t           vers;             // 0x02 in this case
    uint16_t          mpa[TR_DIAG_MAX_MPATH];     // D1
    uint8_t           d_mp[TR_DIAG_MAX_MPATH];    // D1
    uint8_t           ntm;                        // D1
    uint8_t           gpio;                       // C2
    uint8_t           atc;                        // C2
    uint8_t           frame_rate;                 // C2
    uint8_t           trig;                       // C2
    uint8_t           rsvd[3];

} tr_diag_vers_002_t, *p_tr_diag_vers_002_t;


/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/

#endif // TRACKREPORT_H
