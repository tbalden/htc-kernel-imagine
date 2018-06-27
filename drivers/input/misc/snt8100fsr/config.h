/*****************************************************************************
* File: config.h
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
#include <linux/interrupt.h>
#include "firmware.h"
#include "serial_bus.h"
#ifdef CONFIG_HTC_FEATURE
#ifdef CONFIG_FB
#include <linux/fb.h>
#endif
#include <linux/nanohub_htc.h>
#endif

#ifndef CONFIG_H
#define CONFIG_H

#define SNT_VERSION     "Rel.3.2.2"

/*
 * Select which bus to use. Only one bus can be active at a time.
 */
#ifdef CONFIG_NANOHUB_SILEGO
#define USE_SPI_BUS
#endif
//#define USE_I2C_BUS
#define USE_NANOHUB_BUS

/* FIRMWARE_IRQ_TIMEOUT_MS: Amount of time to wait for an interrupt during
 * firmware upload in ms. The timer includes the time to transfer a single
 * payload, so ensure it's long enough.
 */

/*
 *  To support boot-from-flash mode:
 * //#define USE_SPI_BUS
 * #define USE_I2C_BUS
 *
 * in #ifdef USE_I2C_BUS
 * //#define UPLOAD_FIRMWARE
 * #define SUPPORT_FLASH
 * // If you want snt_i2c_write DBUG: message every 512 bytes, uncomment,
 * //#define SNT_I2C_WRITE_DEBUG_VERBOSE
 * //#define SNT_I2C_READ_FIFO_PKT_DEBUG_VERBOSE
 */


/*
 * Custom BUS Settings
 */
#ifdef USE_SPI_BUS
#ifdef CONFIG_NANOHUB_SILEGO
#define SENTONS_SPI_DRIVER_NAME "snt8100fsr-spi"
#else
#define SENTONS_DRIVER_NAME "snt8100fsr-spi"
//#define UPLOAD_FIRMWARE
#define SNT_FWDL_BUF_SIZE         (1024 * 32)
#define FIRMWARE_IRQ_TIMEOUT_MS   2000
#endif
#endif

#ifdef USE_I2C_BUS
#define SENTONS_DRIVER_NAME "snt8100fsr-i2c"
#define CONFIG_I2C_MODULE                   // (used by linux/i2c.h)
#ifdef CONFIG_HTC_FEATURE
//#define UPLOAD_FIRMWARE
#define SUPPORT_FLASH
#define SNT_FWDL_BUF_SIZE         200
#else
#define UPLOAD_FIRMWARE
//#define SUPPORT_FLASH
#define SNT_FWDL_BUF_SIZE         512
#endif  //CONFIG_HTC_FEATURE
#define FIRMWARE_IRQ_TIMEOUT_MS   5000      // supports 100KHz
//#define SNT_I2C_WRITE_DEBUG_VERBOSE
//#define SNT_I2C_READ_FIFO_PKT_DEBUG_VERBOSE
#endif

#ifdef USE_NANOHUB_BUS
#define SENTONS_DRIVER_NAME "snt8100fsr"
//#define UPLOAD_FIRMWARE
#define SUPPORT_FLASH
//#define SNT_FWDL_BUF_SIZE         230		//DEBUG
extern int SNT_FWDL_BUF_SIZE;
#define SNT_FWDL_BUF_RX_SIZE      100
#define FIRMWARE_IRQ_TIMEOUT_MS   15000      // supports 100KHz
#endif

/*
 * if this is commented out then touch reporting will not be enabled by default
 */
//#define TOUCH_ENABLED_AT_STARTUP

/*
 * Set to 1 to use the DTS nodes for configuration. Refer to device.c.
 * If set to 0, then use this config.h file for configuration.
 */
#ifdef CONFIG_OF
#define USE_DEVICE_TREE_CONFIG 1
#else
#define USE_DEVICE_TREE_CONFIG 0
#endif

// SysFS name
#ifdef CONFIG_HTC_FEATURE
#define SYSFS_NAME "android_edge"
#else
#define SYSFS_NAME "snt8100fsr"
#endif

// SySFS permissions for showing, storing, or both
#define SYSFS_PERM_SHOW (S_IRUSR|S_IRUGO)
#define SYSFS_PERM_STORE (S_IWUSR|S_IRUGO)
#define SYSFS_PERM_SHOW_STORE (S_IWUSR|S_IRUSR|S_IRUGO)

// The default frame rate to sample frames from the sensor hardware in hz
#define DEFAULT_FRAME_RATE 100

// The location on disk of the firmware to upload to the hardware on boot
#ifdef CONFIG_HTC_FEATURE
#define FIRMWARE_LOCATION   "snt8100fsr.img"
#else
#define FIRMWARE_LOCATION   "/snt8100fsr.image"
#endif

// The location of the register configuration files, leave a trailing /.
#ifdef CONFIG_HTC_FEATURE
#define REGISTER_FILES_PATH "/persist/snt8100fsr/"
#else
#define REGISTER_FILES_PATH "/tmp/snt8100fsr/"
#endif

/*
 * The prefix sysFS and register configuration files will have, for example,
 * if the prefix is "register_" and the register name is "frame_rate", then
 * the register would be labeled as: "register_frame_rate".
 */
#define REGISTER_PREFIX "register_"


#ifdef CONFIG_HTC_FEATURE
// The location of the track report logfiles
#define TRACK_REPORT_LOG_LOCATION "/data/misc/edge/track_report.log"

// The location of the binary track report log file
#define TRACK_REPORT_BIN_LOG_LOCATION "/data/misc/edge/track_report.bin"

// The location of the deep trace file
#define DEEP_TRACE_LOCATION       "/data/misc/edge/deep_trace.bin"

// The location of the event log file
#define EVENT_LOG_LOCATION        "/data/misc/edge/event_log.bin"

// The location of the small sensor data log file
#define D1TEST_DATA_LOG_LOCATION "/data/misc/edge/d1test_data.log"

// The location of the large sensor data log file
#define FRAME_DATA_LOG_LOCATION "/data/misc/edge/frame_data.log"

// The location of the no touch data log file
#define NO_TOUCH_FRAME_DATA_LOG_LOCATION "/data/misc/edge/no_touch_frame_data.log"
#else
// The location of the track report log file
#define TRACK_REPORT_LOG_LOCATION "/tmp/track_report.log"

// The location of the binary track report log file
#define TRACK_REPORT_BIN_LOG_LOCATION "/tmp/track_report.bin"

// The location of the deep trace file
#define DEEP_TRACE_LOCATION       "/tmp/deep_trace.bin"

// The location of the event log file
#define EVENT_LOG_LOCATION        "/tmp/event_log.bin"

// The location of the small sensor data log file
#define D1TEST_DATA_LOG_LOCATION "/tmp/d1test_data.log"

// The location of the small sensor data log file
#define FRAME_DATA_LOG_LOCATION "/tmp/frame_data.log"

// The location of the no touch data log file
#define NO_TOUCH_FRAME_DATA_LOG_LOCATION "/tmp/no_touch_frame_data.log"
#endif	//CONFIG_HTC_FEATURE

// SPI bus speed settings for firmware update
#ifdef CONFIG_HTC_FEATURE
#define SPI_FLASH_SPEED_KHZ  4800
#endif
// SPI Bus Settings if not loaded via DeviceTree nodes (bottom of file for chart)
#define SPI_MAX_SPEED_HZ    1500000
#define SPI_BUS             1
#define SPI_CHIPSELECT      0

/*
 * The amount of time in microseconds to keep chip select active to wake up
 * the device.
 */
#define SPI_WAKE_CHIP_SELECT_TIME 100

/*
 * the maximum size of an I2c bus transfer. Adjust this to fit OS xfer
 * requirement for i2c.
 */
 #define I2C_MAX_PKT_SIZE_BYTES     512

/*
 * The frame rate to use when the operating system is suspended in hz.
 * Use 5 for Squeeze to Wake mode in which the user squeezes the sensors
 * in order to wake up the device via interrupt.
 * Use 65535 for Deep Sleep in which the sensors will be disabled and the only
 * way to wake up is via sb_wake_device() call, usually made from dev resume()
 *
 * This value is saved into snt8100fsr->suspended_frame_rate and can be
 * updated dynamically at runtime in code and via sysfs in sysfs.c.
 */
#define DEFAULT_SUSPENDED_FRAME_RATE 5
#define DEEP_SLEEP_FRAME_RATE 0xFF

#ifdef CONFIG_HTC_FEATURE
#define FIRMWARE_RETRY_TIMES 0
#endif
// Should we use the IRQ to perform the firmware upload?
#define FIRMWARE_UPLOAD_WITH_IRQ true

// Boot Configuration Record (BCR)
#ifdef CONFIG_HTC_FEATURE
#define BCR_LOGGING_LEVEL   /*BCR_LOGLVL_OFF*/ BCR_LOGLVL_DEFAULT
#else
#define BCR_LOGGING_LEVEL   BCR_LOGLVL_OFF // BCR_LOGLVL_DEFAULT
#endif

// Delay in IRQ'less Firmware Uploads due to Boot Rom LOG Glitch
//#define FIRMWARE_LOG_DELAY_MS 2
#define FIRMWARE_LOG_DELAY_MS 200

// For firmware uploads not using the IRQ, use this delay
#define FIRMWARE_UPLOAD_DELAY_MS 100

// Our interrupt name for the GPIO19 pin interrupt
#ifdef CONFIG_HTC_FEATURE
#define IRQ_NAME            "edge_snt8100fsr_int"
#else
#define IRQ_NAME            "gpio49_int"
#endif

#ifdef CONFIG_HTC_FEATURE
// The GPIO we will use for interrupt handling
extern int IRQ_GPIO;

// The Interrupt number
extern int IRQ_NUMBER;
#else
// This is used to calculate device pin value from the GPIO pin value
#define BEAGLEBONE_GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

// Our two used GPIO pins
#define BEAGLEBONE_GPIO49 BEAGLEBONE_GPIO_TO_PIN(1, 17)

// The GPIO we will use for interrupt handling
#define IRQ_GPIO BEAGLEBONE_GPIO49

// The Interrupt number
#define IRQ_NUMBER gpio_to_irq(IRQ_GPIO)
#endif	//CONFIG_HTC_FEATURE

// The triggers for the interrupt handler to execute on
#define IRQ_TRIGGER IRQF_TRIGGER_RISING

/*
 * From: https://groups.google.com/forum/#!topic/beagleboard/Lo0GEl1RdbU
 * Beagle Bone Black SPI Clock Speed Chart
 * CLKD	Divide by	SPI_CLK [Hz]
 * 0	1       48,000,000
 * 1	2       24,000,000
 * 2	4       12,000,000
 * 3	8       6,000,000
 * 4	16      3,000,000
 * 5	32      1,500,000
 * 6	64      750,000
 * 7	128     375,000
 * 8	256     187,500
 * 9	512     93,750
 * 10	1024	46,875
 * 11	2048	23,438
 * 12	4096	11,719
 * 13	8192	5,859
 * 14	16384	2,930
 * 15	32768	1,465
 */

/*
 * Set up 64 vs 32 bit arch detection. Note this only works for arm right now
 */
#ifdef __aarch64__
#define ENVIRONMENT64
#define PTR_2_UINT uint64_t
#else
#define ENVIRONMENT32
#define PTR_2_UINT  uint32_t
#endif





#endif // CONFIG_H

