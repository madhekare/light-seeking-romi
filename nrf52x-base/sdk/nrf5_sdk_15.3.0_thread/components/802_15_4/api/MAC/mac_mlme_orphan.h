/**
 * Copyright (c) 2016 - 2019 Nordic Semiconductor ASA and Luxoft Global Operations Gmbh.
 *
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef MAC_MLME_ORPHAN_H_INCLUDED
#define MAC_MLME_ORPHAN_H_INCLUDED

#if (CONFIG_ORPHAN_ENABLED == 1)

#include <stdint.h>
#include <stdbool.h>
#include "mac_common.h"

/** @file
 * The MAC MLME Orphan module declares the MAC Orphan routines and
 * necessary types/macros according to the MAC specification.
 *
 * @defgroup mac_orphan MAC MLME Orphan API
 * @ingroup mac_15_4
 * @{
 * @brief Module to declare MAC MLME Orphan API.
 * @details The MAC Orphan module declares routines and necessary types to deal with the Orphan devices
 * according to the MAC specification. More specifically, MAC MLME Orphan indication aka
 * mlme_orphan_ind(), MAC MLME Orphan response aka mlme_orphan_resp() primitives are declared.
 */

/**
 * @brief   MLME-ORPHAN.indication
 *
 * @details The MLME-ORPHAN.indication primitive allows the MLME of a coordinator
 * to notify the next higher layer of the presence of an orphaned device.
 *
 * In accordance with IEEE Std 802.15.4-2006, section 7.1.8.1
 */
typedef struct
{
    /** The address of the orphaned device. */
    uint64_t           orphan_address;
#if (CONFIG_SECURE == 1)
    uint8_t            security_level;       /**< Security level. */
    uint8_t            key_id_mode;          /**< Key ID mode. */
    uint64_t           key_source;           /**< Key source. */
    uint8_t            key_index;            /**< Key index. */
#endif
} mlme_orphan_ind_t;


/**
 * @brief   MLME-ORPHAN.response
 *
 * @details The MLME-ORPHAN.response primitive allows the next higher layer of a coordinator
 * to respond to the MLME-ORPHAN.indication primitive.
 *
 * In accordance with IEEE Std 802.15.4-2006, section 7.1.8.2
 */
typedef struct
{
    /** Do not edit this field. */
    mac_abstract_req_t service;

    /** The address of the orphaned device. */
    uint64_t           orphan_address;

    /**
     * The 16-bit short address allocated to the orphaned device if it is associated with this
     * coordinator. The special short address 0xfffe indicates that no short address was
     * allocated, and the device will use its 64-bit extended address in all communications.
     * If the device was not associated with this coordinator, this field will contain the
     * value 0xffff and be ignored on receipt.
     */
    uint16_t           short_address;

    /** TRUE if the orphaned device is associated with this coordinator or FALSE otherwise. */
    bool               associated_member;
#if (CONFIG_SECURE == 1)
    uint8_t            security_level;       /**< Security level. */
    uint8_t            key_id_mode;          /**< Key ID mode. */
    uint64_t           key_source;           /**< Key source. */
    uint8_t            key_index;            /**< Key index. */
#endif
} mlme_orphan_resp_t;


/**
 * @brief   MLME-ORPHAN.indication handler
 *
 * @details The MLME-ORPHAN.indication primitive is generated by the MLME of a coordinator
 * and issued to its next higher layer on receipt of an orphan notification command (see 7.3.6).
 *
 * @param ind  MLME-ORPHAN.indication structure.
 *
 * In accordance with IEEE Std 802.15.4-2006, section 7.1.8.1
 */
extern void mlme_orphan_ind(mlme_orphan_ind_t * ind);


/**
 * @brief   MLME-ORPHAN.response handler
 *
 * @details The MLME-ORPHAN.response primitive is generated by the next higher layer and issued to its MLME
 * when it reaches a decision about whether the orphaned device indicated in the MLME-ORPHAN.indication
 * primitive is associated.
 *
 * @param resp  MLME-ORPHAN.response structure.
 *
 * In accordance with IEEE Std 802.15.4-2006, section 7.1.8.2
 */
void mlme_orphan_resp(mlme_orphan_resp_t * resp);

/** @} */

#endif // (CONFIG_ORPHAN_ENABLED == 1)

#endif // MAC_MLME_ORPHAN_H_INCLUDED
