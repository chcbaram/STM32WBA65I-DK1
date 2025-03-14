#ifndef __MP95P32_H
#define __MP95P32_H

#ifdef __cplusplus
 extern "C" {
#endif


#define MP95P32_FLASH_SIZE                  (0x0400000)    /* 32 MBits => 4MBytes */
#define MP95P32_SECTOR_SIZE                 0x10000        /* 64KBytes */
#define MP95P32_SUBSECTOR_SIZE              0x1000         /* 4kBytes */
#define MP95P32_PAGE_SIZE                   0x200          /* 512 bytes */

#define MP95P32_DUMMY_CYCLES_READ           8
#define MP95P32_DUMMY_CYCLES_READ_QUAD      4

#define MP95P32_BULK_ERASE_MAX_TIME         250000
#define MP95P32_SECTOR_ERASE_MAX_TIME       3000
#define MP95P32_SUBSECTOR_ERASE_MAX_TIME    800

/**
  * @brief  MP95P32 Commands
  */
/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

/* Identification Operations */
#define READ_ID_CMD                          0x83

/* Read Operations */
#define READ_CMD                             0x03
#define FAST_READ_CMD                        0x0B
#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_INOUT_FAST_READ_CMD             0xEB

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define WRITE_STATUS_REG_CMD                 0x01

#define READ_STATUS_REG2_CMD                 0x35
#define WRITE_STATUS_REG2_CMD                0x31

#define READ_STATUS_REG3_CMD                 0x15
#define WRITE_STATUS_REG3_CMD                0x11


#define READ_LOCK_REG_CMD                    0xE8
#define WRITE_LOCK_REG_CMD                   0xE5

#define READ_FLAG_STATUS_REG_CMD             0x70
#define CLEAR_FLAG_STATUS_REG_CMD            0x50

#define READ_NONVOL_CFG_REG_CMD              0xB5
#define WRITE_NONVOL_CFG_REG_CMD             0xB1

#define READ_VOL_CFG_REG_CMD                 0x85
#define WRITE_VOL_CFG_REG_CMD                0x81

#define READ_ENHANCED_VOL_CFG_REG_CMD        0x65
#define WRITE_ENHANCED_VOL_CFG_REG_CMD       0x61

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define DUAL_IN_FAST_PROG_CMD                0xA2
#define EXT_DUAL_IN_FAST_PROG_CMD            0xD2
#define QUAD_IN_FAST_PROG_CMD                0x32
#define EXT_QUAD_IN_FAST_PROG_CMD            0x12

/* Erase Operations */
#define SUBSECTOR_ERASE_CMD                  0x20
#define SECTOR_ERASE_CMD                     0xD8
#define BULK_ERASE_CMD                       0xC7

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* One-Time Programmable Operations */
#define READ_OTP_ARRAY_CMD                   0x4B
#define PROG_OTP_ARRAY_CMD                   0x42

/**
  * @brief  MP95P32 Registers
  */
/* Status Register */
#define MP95P32_SR_WIP                      ((uint8_t)0x01)    /*!< Write in progress */
#define MP95P32_SR_WREN                     ((uint8_t)0x02)    /*!< Write enable latch */
#define MP95P32_SR_BLOCKPR                  ((uint8_t)0x5C)    /*!< Block protected against program and erase operations */
#define MP95P32_SR_PRBOTTOM                 ((uint8_t)0x20)    /*!< Protected memory area defined by BLOCKPR starts from top or bottom */
#define MP95P32_SR_SRWREN                   ((uint8_t)0x80)    /*!< Status register write enable/disable */

/* Nonvolatile Configuration Register */
#define MP95P32_NVCR_LOCK                   ((uint16_t)0x0001) /*!< Lock nonvolatile configuration register */
#define MP95P32_NVCR_DUAL                   ((uint16_t)0x0004) /*!< Dual I/O protocol */
#define MP95P32_NVCR_QUAB                   ((uint16_t)0x0008) /*!< Quad I/O protocol */
#define MP95P32_NVCR_RH                     ((uint16_t)0x0010) /*!< Reset/hold */
#define MP95P32_NVCR_ODS                    ((uint16_t)0x01C0) /*!< Output driver strength */
#define MP95P32_NVCR_XIP                    ((uint16_t)0x0E00) /*!< XIP mode at power-on reset */
#define MP95P32_NVCR_NB_DUMMY               ((uint16_t)0xF000) /*!< Number of dummy clock cycles */

/* Volatile Configuration Register */
#define MP95P32_VCR_WRAP                    ((uint8_t)0x03)    /*!< Wrap */
#define MP95P32_VCR_XIP                     ((uint8_t)0x08)    /*!< XIP */
#define MP95P32_VCR_NB_DUMMY                ((uint8_t)0xF0)    /*!< Number of dummy clock cycles */

/* Enhanced Volatile Configuration Register */
#define MP95P32_EVCR_ODS                    ((uint8_t)0x07)    /*!< Output driver strength */
#define MP95P32_EVCR_VPPA                   ((uint8_t)0x08)    /*!< Vpp accelerator */
#define MP95P32_EVCR_RH                     ((uint8_t)0x10)    /*!< Reset/hold */
#define MP95P32_EVCR_DUAL                   ((uint8_t)0x40)    /*!< Dual I/O protocol */
#define MP95P32_EVCR_QUAD                   ((uint8_t)0x80)    /*!< Quad I/O protocol */

/* Flag Status Register */
#define MP95P32_FSR_PRERR                   ((uint8_t)0x02)    /*!< Protection error */
#define MP95P32_FSR_PGSUS                   ((uint8_t)0x04)    /*!< Program operation suspended */
#define MP95P32_FSR_VPPERR                  ((uint8_t)0x08)    /*!< Invalid voltage during program or erase */
#define MP95P32_FSR_PGERR                   ((uint8_t)0x10)    /*!< Program error */
#define MP95P32_FSR_ERERR                   ((uint8_t)0x20)    /*!< Erase error */
#define MP95P32_FSR_ERSUS                   ((uint8_t)0x40)    /*!< Erase operation suspended */
#define MP95P32_FSR_READY                   ((uint8_t)0x80)    /*!< Ready or command in progress */




#ifdef __cplusplus
}
#endif

#endif /* __MP95P32_H */
