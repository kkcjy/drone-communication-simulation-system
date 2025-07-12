#ifndef RANGING_DEFCONFIG_H
#define RANGING_DEFCONFIG_H

// #define         CLASSIC_TOF_ENABLE
#define         COORDINATE_SEND_ENABLE
// #define         COMPENSATE_ENABLE
#define         STATE_MACHINE_ENABLE

#define         COMPENSATE_RATE             0.5

#define         NULL_ADDR                   0xFFFF
#define         NULL_SEQ                    0x0
#define         NULL_TIMESTAMP              0xFFFFFFFFU
#define         NULL_INDEX                  0xFF
#define         NULL_TOF                    -1.0f
#define         INCOMPLETE_SIGN             -2.0f
#define         MISORDER_SIGN               -3.0f

#define         table_index_t               uint8_t
#define         index_t                     uint8_t

#define         MESSAGE_TX_POOL_SIZE        3
#define         MESSAGE_BODY_UNIT_SIZE      2

#define         SEND_LIST_SIZE              5
#define         RANGING_TABLE_SIZE          10        

#define         UPPER_BOUND_DISTANCE        1000
#define         LOWER_BOUND_DISTANCE        0

#define         RANGING_PERIOD              200
#define         CHECK_PERIOD                15
#define         UWB_MAX_TIMESTAMP           1099511627776
#define         VELOCITY                    0.4691763978616
#define         CONVERGENCE_THRESHOLD       0.989
#define         INIT_CALCULATION_ROUNDS     4

#define         M2T(X)                      ((unsigned int)(X))

#endif