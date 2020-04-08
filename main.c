/***************************************************************************//**
 * @file
 * @brief Silicon Labs Bluetooth mesh light switch example
 * This example implements a Bluetooth mesh light switch.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "retargetserial.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>
#include <em_rtcc.h>
#include <gpiointerrupt.h>

/* Coex header */
#include "coexistence-ble.h"

/* Device initialization header */
#include "hal-config.h"

/* Display Interface header */
#include "display_interface.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#ifdef ENABLE_LOGGING
#define log(...) printf(__VA_ARGS__)
#else
#define log(...)
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/// Maximum number of simultaneous Bluetooth connections
#define MAX_CONNECTIONS 2

/// Heap for Bluetooth stack
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

/// Bluetooth advertisement set configuration
///
/// At minimum the following is required:
/// * One advertisement set for Bluetooth LE stack (handle number 0)
/// * One advertisement set for Mesh data (handle number 1)
/// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
/// * One advertisement set for Mesh unprovisioned URI (handle number 3)
/// * N advertisement sets for Mesh GATT service advertisements
/// (one for each network key, handle numbers 4 .. N+3)
///
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

/// Priorities for bluetooth link layer operations
static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

/// Bluetooth stack configuration
const gecko_configuration_t config =
{
#if defined(FEATURE_LFXO) || defined(PLFRCO_PRESENT) || defined(LFRCO_PRESENT)
// Disable sleep on the following boards to make buttons's interrupts work.
// If you enable sleep on these boards, WSTK pushbuttons will not wake up chip.
// Only Ports A and B support EM2 wake-up. Please refer to EFR32xG21 reference manual GPIO chapter.
#if defined(BRD4180A) || defined(BRD4181A)
  .sleep.flags = 0,
#else
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
#endif
#else
  .sleep.flags = 0,
#endif
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
#if defined(FEATURE_LFXO)
  .bluetooth.sleep_clock_accuracy = 100, // ppm
#elif defined(PLFRCO_PRESENT) || defined(LFRCO_PRESENT)
  .bluetooth.sleep_clock_accuracy = 500, // ppm
#endif
  .bluetooth.linklayer_priorities = &linklayer_priorities,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
  .max_timers = 16,
  .rf.flags = GECKO_RF_CONFIG_ANTENNA,   // Enable antenna configuration.
  .rf.antenna = GECKO_RF_ANTENNA,   // Select antenna path!
};

/// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

/// Timer Frequency used
#define TIMER_CLK_FREQ ((uint32_t)32768)
/// Convert miliseconds to timer ticks
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)
/** Stop timer. */
#define TIMER_STOP 0

/*******************************************************************************
 * Timer handles defines.
 ******************************************************************************/
#define TIMER_ID_RESTART            78
#define TIMER_ID_FACTORY_RESET      77
#define TIMER_ID_PROVISIONING       66
#define TIMER_ID_RETRANS_ONOFF      10
#define TIMER_ID_RETRANS_LIGHTNESS  11
#define TIMER_ID_RETRANS_CTL        12
#define TIMER_ID_RETRANS_SCENE      13
#define TIMER_ID_FRIEND_FIND        20
#define TIMER_ID_NODE_CONFIGURED    30

/// Minimum color temperature 800K
#define TEMPERATURE_MIN      0x0320
/// Maximum color temperature 20000K
#define TEMPERATURE_MAX      0x4e20
/// Delta UV is hardcoded to 0 in this example
#define DELTA_UV  0

#define IMMEDIATE          0 ///< Immediate transition time is 0 seconds
#define PUBLISH_ADDRESS    0 ///< The unused 0 address is used for publishing
#define IGNORED            0 ///< Parameter ignored for publishing
#define NO_FLAGS           0 ///< No flags used for message

/*******************************************************************************
 * Global variables
 ******************************************************************************/
/// For indexing elements of the node (this example has only one element)
static uint16_t _elem_index = 0xffff;
/// Address of the Primary Element of the Node
static uint16_t _my_address = 0;
/// current position of the switch
static uint8_t switch_pos = 0;
/// number of on/off requests to be sent
static uint8_t onoff_request_count;
/// on/off transaction identifier
static uint8_t onoff_trid = 0;
/// currently selected scene
static uint16_t scene_number = 0;
/// number of scene requests to be sent
static uint8_t scene_request_count;
/// scene transaction identifier
static uint8_t scene_trid = 0;
/// number of active Bluetooth connections
static uint8_t num_connections = 0;
/// handle of the last opened LE connection
static uint8_t conn_handle = 0xFF;
/// Flag for indicating that lpn feature is active
static uint8_t lpn_active = 0;

/// lightness level percentage
static uint8_t lightness_percent = 0;
/// lightness level converted from percentage to actual value, range 0..65535
static uint16_t lightness_level = 0;
/// number of lightness requests to be sent
static uint8_t lightness_request_count;
/// lightness transaction identifier
static uint8_t lightness_trid = 0;
/// temperature level percentage
static uint8_t temperature_percent = 50;
/// temperature level converted from percentage to actual value, range 0..65535
static uint16_t temperature_level = 0;
/// number of ctl requests to be sent
static uint8_t ctl_request_count;
/// ctl transaction identifier
static uint8_t ctl_trid = 0;

/// button press timestamp for very long/long/medium/short Push Button 0 press detection
static uint32_t pb0_press;
/// button press timestamp for very long/long/medium/short Push Button 1 press detection
static uint32_t pb1_press;

/// Number of ticks after which press is considered to be medium (0.25s)
#define TICKS_FOR_250_MILLISECONDS   TIMER_MS_2_TIMERTICK(250)
/// Number of ticks after which press is considered to be long (1s)
#define TICKS_FOR_1_SECOND           TIMER_MS_2_TIMERTICK(1000)
/// Number of ticks after which press is considered to be very long (5s)
#define TICKS_FOR_5_SECONDS          TIMER_MS_2_TIMERTICK(5000)

/*******************************************************************************
 * External signal definitions. These are used to signal button press events
 * from GPIO interrupt handler to application.
 ******************************************************************************/
#define EXT_SIGNAL_PB0_SHORT_PRESS       0x01
#define EXT_SIGNAL_PB1_SHORT_PRESS       0x02
#define EXT_SIGNAL_PB0_MEDIUM_PRESS      0x04
#define EXT_SIGNAL_PB1_MEDIUM_PRESS      0x08
#define EXT_SIGNAL_PB0_LONG_PRESS        0x10
#define EXT_SIGNAL_PB1_LONG_PRESS        0x20
#define EXT_SIGNAL_PB0_VERY_LONG_PRESS   0x40
#define EXT_SIGNAL_PB1_VERY_LONG_PRESS   0x80

/*******************************************************************************
 *  State of the LEDs is updated by calling LED_set_state().
 *  The new state is passed as parameter, possible values are defined below.
 ******************************************************************************/
#define LED_STATE_OFF    0   ///< light off (both LEDs turned off)
#define LED_STATE_ON     1   ///< light on (both LEDs turned on)
#define LED_STATE_PROV   3   ///< provisioning (LEDs blinking)

/*******************************************************************************
 *  These defines are needed to support radio boards with active-low and
 *  active-high LED configuration.
 ******************************************************************************/
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
/* LED GPIO is active-low */
#define TURN_LED_OFF   GPIO_PinOutSet
#define TURN_LED_ON    GPIO_PinOutClear
#define LED_DEFAULT_STATE  1
#else
/* LED GPIO is active-high */
#define TURN_LED_OFF   GPIO_PinOutClear
#define TURN_LED_ON    GPIO_PinOutSet
#define LED_DEFAULT_STATE  0
#endif

/***************************************************************************//**
 * Update the state of LEDs.
 *
 * @param[in] state  New state defined as LED_STATE_xxx.
 ******************************************************************************/
static void LED_set_state(uint8_t state)
{
  switch (state) {
    case LED_STATE_OFF:
      TURN_LED_OFF(BSP_LED0_PORT, BSP_LED0_PIN);
      TURN_LED_OFF(BSP_LED1_PORT, BSP_LED1_PIN);
      break;
    case LED_STATE_ON:
      TURN_LED_ON(BSP_LED0_PORT, BSP_LED0_PIN);
      TURN_LED_ON(BSP_LED1_PORT, BSP_LED1_PIN);
      break;
    case LED_STATE_PROV:
      GPIO_PinOutToggle(BSP_LED0_PORT, BSP_LED0_PIN);
      GPIO_PinOutToggle(BSP_LED1_PORT, BSP_LED1_PIN);
      break;

    default:
      break;
  }
}

/***************************************************************************//**
 * This is a callback function that is invoked each time a GPIO interrupt
 * in one of the pushbutton inputs occurs. Pin number is passed as parameter.
 *
 * @param[in] pin  Pin number where interrupt occurs
 *
 * @note This function is called from ISR context and therefore it is
 *       not possible to call any BGAPI functions directly. The button state
 *       change is signaled to the application using gecko_external_signal()
 *       that will generate an event gecko_evt_system_external_signal_id
 *       which is then handled in the main loop.
 ******************************************************************************/
void gpioint(uint8_t pin)
{
  uint32_t t_diff;

  if (pin == BSP_BUTTON0_PIN) {
    if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0) {
      // PB0 pressed - record RTCC timestamp
      pb0_press = RTCC_CounterGet();
    } else {
      // PB0 released - check if it was short, medium, long or very long press
      t_diff = RTCC_CounterGet() - pb0_press;
      if (t_diff < TICKS_FOR_250_MILLISECONDS) {
        gecko_external_signal(EXT_SIGNAL_PB0_SHORT_PRESS);
      } else if (t_diff < TICKS_FOR_1_SECOND) {
        gecko_external_signal(EXT_SIGNAL_PB0_MEDIUM_PRESS);
      } else if (t_diff < TICKS_FOR_5_SECONDS) {
        gecko_external_signal(EXT_SIGNAL_PB0_LONG_PRESS);
      } else {
        gecko_external_signal(EXT_SIGNAL_PB0_VERY_LONG_PRESS);
      }
    }
  } else if (pin == BSP_BUTTON1_PIN) {
    if (GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0) {
      // PB1 pressed - record RTCC timestamp
      pb1_press = RTCC_CounterGet();
    } else {
      // PB1 released - check if it was short, medium, long or very long press
      t_diff = RTCC_CounterGet() - pb1_press;
      if (t_diff < TICKS_FOR_250_MILLISECONDS) {
        gecko_external_signal(EXT_SIGNAL_PB1_SHORT_PRESS);
      } else if (t_diff < TICKS_FOR_1_SECOND) {
        gecko_external_signal(EXT_SIGNAL_PB1_MEDIUM_PRESS);
      } else if (t_diff < TICKS_FOR_5_SECONDS) {
        gecko_external_signal(EXT_SIGNAL_PB1_LONG_PRESS);
      } else {
        gecko_external_signal(EXT_SIGNAL_PB1_VERY_LONG_PRESS);
      }
    }
  }
}

/***************************************************************************//**
 * Enable button interrupts for PB0, PB1. Both GPIOs are configured to trigger
 * an interrupt on the rising edge (button released).
 ******************************************************************************/
void enable_button_interrupts(void)
{
  GPIOINT_Init();

  /* configure interrupt for PB0 and PB1, both falling and rising edges */
  GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, true, true);
  GPIO_ExtIntConfig(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, BSP_BUTTON1_PIN, true, true, true);

  /* register the callback function that is invoked when interrupt occurs */
  GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, gpioint);
  GPIOINT_CallbackRegister(BSP_BUTTON1_PIN, gpioint);
}

/***************************************************************************//**
 * This function publishes one generic on/off request to change the state
 * of light(s) in the group. Global variable switch_pos holds the latest
 * desired light state, possible values are:
 * switch_pos = 1 -> PB1 was pressed long (above 1s), turn lights on
 * switch_pos = 0 -> PB0 was pressed long (above 1s), turn lights off
 *
 * param[in] retrans  Indicates if this is the first request or a retransmission,
 *                    possible values are 0 = first request, 1 = retransmission.
 *
 * @note This application sends multiple generic on/off requests for each
 *       long button press to improve reliability.
 *       The transaction ID is not incremented in case of a retransmission.
 ******************************************************************************/
void send_onoff_request(uint8_t retrans)
{
  struct mesh_generic_request req;
  const uint32_t transtime = 0; // using zero transition time by default

  req.kind = mesh_generic_request_on_off;
  req.on_off = switch_pos ? MESH_GENERIC_ON_OFF_STATE_ON : MESH_GENERIC_ON_OFF_STATE_OFF;

  // increment transaction ID for each request, unless it's a retransmission
  if (retrans == 0) {
    onoff_trid++;
  }

  // Delay for the request is calculated so that the last request will have
  // a zero delay and each of the previous request have delay that increases
  // in 50 ms steps. For example, when using three on/off requests
  // per button press the delays are set as 100, 50, 0 ms
  uint16_t delay = (onoff_request_count - 1) * 50;

  uint16_t resp = mesh_lib_generic_client_publish(MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
                                                  _elem_index,
                                                  onoff_trid,
                                                  &req,
                                                  transtime, // transition time in ms
                                                  delay,
                                                  NO_FLAGS   // flags
                                                  );

  if (resp) {
    log("gecko_cmd_mesh_generic_client_publish failed, code 0x%x\r\n", resp);
  } else {
    log("on/off request sent, trid = %u, delay = %u\r\n", onoff_trid, delay);
  }

  // Keep track of how many requests has been sent
  if (onoff_request_count > 0) {
    onoff_request_count--;
  }
}

/***************************************************************************//**
 * This function publishes one light lightness request to change the lightness
 * level of light(s) in the group. Global variable lightness_level holds
 * the latest desired light level.
 *
 * param[in] retrans  Indicates if this is the first request or a retransmission,
 *                    possible values are 0 = first request, 1 = retransmission.
 *
 * @note This application sends multiple lightness requests for each
 *       short button press to improve reliability.
 *       The transaction ID is not incremented in case of a retransmission.
 ******************************************************************************/
void send_lightness_request(uint8_t retrans)
{
  struct mesh_generic_request req;

  req.kind = mesh_lighting_request_lightness_actual;
  req.lightness = lightness_level;

  // increment transaction ID for each request, unless it's a retransmission
  if (retrans == 0) {
    lightness_trid++;
  }

  // Delay for the request is calculated so that the last request will have
  // a zero delay and each of the previous request have delay that increases
  // in 50 ms steps. For example, when using three lightness requests
  // per button press the delays are set as 100, 50, 0 ms
  uint16_t delay = (lightness_request_count - 1) * 50;

  uint16_t resp = mesh_lib_generic_client_publish(MESH_LIGHTING_LIGHTNESS_CLIENT_MODEL_ID,
                                                  _elem_index,
                                                  lightness_trid,
                                                  &req,
                                                  IMMEDIATE,     // transition
                                                  delay,
                                                  NO_FLAGS       // flags
                                                  );

  if (resp) {
    log("gecko_cmd_mesh_generic_client_publish failed, code 0x%x\r\n", resp);
  } else {
    log("lightness request sent, trid = %u, delay = %u\r\n",
        lightness_trid,
        delay);
  }

  // Keep track of how many requests has been sent
  if (lightness_request_count > 0) {
    lightness_request_count--;
  }
}

/***************************************************************************//**
 * This function publishes one light CTL request to change the temperature level
 * of light(s) in the group. Global variable temperature_level holds the latest
 * desired light temperature level.
 * The CTL request also send lightness_level which holds the latest desired light
 * lightness level and Delta UV which is hardcoded to 0 for this application.
 *
 * param[in] retrans  Indicates if this is the first request or a retransmission,
 *                    possible values are 0 = first request, 1 = retransmission.
 *
 * @note This application sends multiple ctl requests for each
 *       medium button press to improve reliability.
 *       The transaction ID is not incremented in case of a retransmission.
 ******************************************************************************/
void send_ctl_request(uint8_t retrans)
{
  struct mesh_generic_request req;

  req.kind = mesh_lighting_request_ctl;
  req.ctl.lightness = lightness_level;
  req.ctl.temperature = temperature_level;
  req.ctl.deltauv = DELTA_UV; //hardcoded delta uv

  // increment transaction ID for each request, unless it's a retransmission
  if (retrans == 0) {
    ctl_trid++;
  }

  // Delay for the request is calculated so that the last request will have
  // a zero delay and each of the previous request have delay that increases
  // in 50 ms steps. For example, when using three ctl requests
  // per button press the delays are set as 100, 50, 0 ms
  uint16_t delay = (ctl_request_count - 1) * 50;

  uint16_t resp = mesh_lib_generic_client_publish(MESH_LIGHTING_CTL_CLIENT_MODEL_ID,
                                                  _elem_index,
                                                  ctl_trid,
                                                  &req,
                                                  IMMEDIATE,     // transition
                                                  delay,
                                                  NO_FLAGS       // flags
                                                  );

  if (resp) {
    log("gecko_cmd_mesh_generic_client_publish failed, code 0x%x\r\n", resp);
  } else {
    log("ctl request sent, trid = %u, delay = %u\r\n", ctl_trid, delay);
  }

  // Keep track of how many requests has been sent
  if (ctl_request_count > 0) {
    ctl_request_count--;
  }
}

/***************************************************************************//**
 * This function publishes one scene recall request to recall selected scene.
 * Global variable scene_number holds the latest desired scene state.
 *
 * param[in] retrans  Indicates if this is the first request or a retransmission,
 *                    possible values are 0 = first request, 1 = retransmission.
 *
 * @note This application sends multiple scene requests for each
 *       very long button press to improve reliability.
 *       The transaction ID is not incremented in case of a retransmission.
 ******************************************************************************/
void send_scene_recall_request(uint8_t retrans)
{
  // Increment transaction ID for each request, unless it's a retransmission
  if (retrans == 0) {
    scene_trid++;
  }

  // Delay for the request is calculated so that the last request will have
  // a zero delay and each of the previous request have delay that increases
  // in 50 ms steps. For example, when using three scene requests
  // per button press the delays are set as 100, 50, 0 ms
  uint16_t delay = (scene_request_count - 1) * 50;

  uint16_t resp = gecko_cmd_mesh_scene_client_recall(_elem_index,
                                                     PUBLISH_ADDRESS,
                                                     IGNORED,
                                                     NO_FLAGS,
                                                     scene_number,
                                                     scene_trid,
                                                     IMMEDIATE,
                                                     delay
                                                     )->result;

  if (resp) {
    log("gecko_cmd_scene_client_recall failed, code 0x%x\r\n", resp);
  } else {
    log("scene request sent, trid = %u, delay = %u\r\n", scene_trid, delay);
  }

  // Keep track of how many requests has been sent
  if (scene_request_count > 0) {
    scene_request_count--;
  }
}

/***************************************************************************//**
 * Handling of short button presses (less than 0.25s).
 * This function is called from the main loop when application receives
 * event gecko_evt_system_external_signal_id.
 *
 * @param[in] button  Defines which button was pressed,
 *                    possible values are 0 = PB0, 1 = PB1.
 *
 * @note This function is called from application context (not ISR)
 *       so it is safe to call BGAPI functions
 ******************************************************************************/
void handle_short_press(uint8_t button)
{
  // short press adjusts light brightness, using Light Lightness model
  if (button == 1) {
    lightness_percent += 10;
    if (lightness_percent > 100) {
      lightness_percent = 100;
    }
  } else {
    if (lightness_percent >= 10) {
      lightness_percent -= 10;
    }
  }

  lightness_level = lightness_percent * 0xFFFF / 100;
  log("set light to %u %% / level %u\r\n", lightness_percent, lightness_level);

  lightness_request_count = 3; // Request is sent 3 times to improve reliability

  send_lightness_request(0);  // Send the first request

  // If there are more requests to send, start a repeating soft timer
  // to trigger retransmission of the request after 50 ms delay
  if (lightness_request_count > 0) {
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(50),
                                      TIMER_ID_RETRANS_LIGHTNESS,
                                      0);
  }
}

/***************************************************************************//**
 * Handling of medium button presses (greater than 0.25s and less than 1s).
 * This function is called from the main loop when application receives
 * event gecko_evt_system_external_signal_id.
 *
 * @param[in] button  Defines which button was pressed,
 *                    possible values are 0 = PB0, 1 = PB1.
 *
 * @note This function is called from application context (not ISR)
 *       so it is safe to call BGAPI functions
 ******************************************************************************/
void handle_medium_press(uint8_t button)
{
  // medium press adjusts light temperature, using Light CTL model
  if (button == 1) {
    temperature_percent += 10;
    if (temperature_percent > 100) {
      temperature_percent = 100;
    }
  } else {
    if (temperature_percent >= 10) {
      temperature_percent -= 10;
    }
  }

  // using square of percentage to change temperature more uniformly just for demonstration
  temperature_level = TEMPERATURE_MIN + (temperature_percent * temperature_percent / 100) * (TEMPERATURE_MAX - TEMPERATURE_MIN) / 100;
  log("set temperature to %u %% / level %u K\r\n",
      temperature_percent * temperature_percent / 100,
      temperature_level);

  ctl_request_count = 3; // Request is send 3 times to improve reliability

  send_ctl_request(0);  //Send the first request

  // If there are more requests to send, start a repeating soft timer
  // to trigger retransmission of the request after 50 ms delay
  if (ctl_request_count > 0) {
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(50),
                                      TIMER_ID_RETRANS_CTL,
                                      0);
  }
}

/***************************************************************************//**
 * Handling of long button presses (greater than 1s).
 * This function is called from the main loop when application receives
 * event gecko_evt_system_external_signal_id.
 *
 * @param[in] button  Defines which button was pressed,
 *                    possible values are 0 = PB0, 1 = PB1.
 *
 * @note This function is called from application context (not ISR),
 *       so it is safe to call BGAPI functions
 ******************************************************************************/
void handle_long_press(uint8_t button)
{
  // PB0 -> switch off, PB1 -> switch on
  switch_pos = button;

  // long press turns light ON or OFF, using Generic OnOff model
  log("PB%u -> turn light(s) ", button);
  if (switch_pos) {
    log("on\r\n");
    lightness_percent = 100;
  } else {
    log("off\r\n");
    lightness_percent = 0;
  }

  onoff_request_count = 3; // Request is sent 3 times to improve reliability

  send_onoff_request(0);  // Send the first request

  // If there are more requests to send, start a repeating soft timer
  // to trigger retransmission of the request after 50 ms delay
  if (onoff_request_count > 0) {
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(50),
                                      TIMER_ID_RETRANS_ONOFF,
                                      0);
  }
}

/***************************************************************************//**
 * Handling of very long button presses (greater than 5s).
 * This function is called from the main loop when application receives
 * event gecko_evt_system_external_signal_id.
 *
 * @param[in] button  Defines which button was pressed,
 *                    possible values are 0 = PB0, 1 = PB1.
 *
 * @note This function is called from application context (not ISR),
 *       so it is safe to call BGAPI functions
 ******************************************************************************/
void handle_very_long_press(uint8_t button)
{
  // PB0 -> scene number 1, PB1 -> scene number 2
  scene_number = button + 1;

  // Very long press recall scene 1 or 2, using Scene Client model
  log("PB%u -> recall scene number %u\r\n", button, scene_number);

  scene_request_count = 3; // Request is sent 3 times to improve reliability

  send_scene_recall_request(0);  // Send the first request

  // If there are more requests to send, start a repeating soft timer
  // to trigger retransmission of the request after 50 ms delay
  if (scene_request_count > 0) {
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(50),
                                      TIMER_ID_RETRANS_SCENE,
                                      0);
  }
}

/***************************************************************************//**
 * Initialize LPN functionality with configuration and friendship establishment.
 ******************************************************************************/
void lpn_init(void)
{
  uint16_t result;

  // Do not initialize LPN if lpn is currently active
  // or any GATT connection is opened
  if (lpn_active || num_connections) {
    return;
  }

  // Initialize LPN functionality.
  result = gecko_cmd_mesh_lpn_init()->result;
  if (result) {
    log("LPN init failed (0x%x)\r\n", result);
    return;
  }
  lpn_active = 1;
  log("LPN initialized\r\n");
  DI_Print("LPN on", DI_ROW_LPN);

  // Configure LPN Minimum friend queue length = 2
  result = gecko_cmd_mesh_lpn_config(mesh_lpn_queue_length, 2)->result;
  if (result) {
    log("LPN queue configuration failed (0x%x)\r\n", result);
    return;
  }
  // Configure LPN Poll timeout = 5 seconds
  result = gecko_cmd_mesh_lpn_config(mesh_lpn_poll_timeout, 5 * 1000)->result;
  if (result) {
    log("LPN Poll timeout configuration failed (0x%x)\r\n", result);
    return;
  }
  log("trying to find friend...\r\n");
  result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

  if (result != 0) {
    log("ret.code 0x%x\r\n", result);
  }
}

/***************************************************************************//**
 * Deinitialize LPN functionality.
 ******************************************************************************/
void lpn_deinit(void)
{
  uint16_t result;

  if (!lpn_active) {
    return; // lpn feature is currently inactive
  }

  // Cancel friend finding timer
  result = gecko_cmd_hardware_set_soft_timer(TIMER_STOP,
                                             TIMER_ID_FRIEND_FIND,
                                             1)->result;

  // Terminate friendship if exist
  result = gecko_cmd_mesh_lpn_terminate_friendship()->result;
  if (result) {
    log("Friendship termination failed (0x%x)\r\n", result);
  }
  // turn off lpn feature
  result = gecko_cmd_mesh_lpn_deinit()->result;
  if (result) {
    log("LPN deinit failed (0x%x)\r\n", result);
  }
  lpn_active = 0;
  log("LPN deinitialized\r\n");
  DI_Print("LPN off", DI_ROW_LPN);
}

/***************************************************************************//**
 * Switch node initialization.
 * This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 ******************************************************************************/
void switch_node_init(void)
{
  // Initialize mesh lib, up to 8 models
  mesh_lib_init(malloc, free, 8);
}

/***************************************************************************//**
 * Handling of stack events. Both Bluetooth LE and Bluetooth mesh events
 * are handled here.
 * @param[in] evt_id  Incoming event ID.
 * @param[in] evt     Pointer to incoming event.
 ******************************************************************************/
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

/***************************************************************************//**
 * Button initialization. Configure pushbuttons PB0,PB1 as inputs.
 ******************************************************************************/
static void button_init(void)
{
  // configure pushbutton PB0 and PB1 as inputs, with pull-up enabled
  GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPull, 1);
  GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInputPull, 1);
}

/***************************************************************************//**
 * LED initialization. Configure LED pins as outputs.
 ******************************************************************************/
static void led_init(void)
{
  // configure LED0 and LED1 as outputs
  GPIO_PinModeSet(BSP_LED0_PORT, BSP_LED0_PIN, gpioModePushPull, LED_DEFAULT_STATE);
  GPIO_PinModeSet(BSP_LED1_PORT, BSP_LED1_PIN, gpioModePushPull, LED_DEFAULT_STATE);
}

/***************************************************************************//**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 *
 * @param[in] pAddr  Pointer to Bluetooth address.
 ******************************************************************************/
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16_t res;

  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "switch node %02x:%02x", pAddr->addr[1], pAddr->addr[0]);

  log("Device name: '%s'\r\n", name);

  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8_t *)name)->result;
  if (res) {
    log("gecko_cmd_gatt_server_write_attribute_value() failed, code 0x%x\r\n", res);
  }

  // show device name on the LCD
  DI_Print(name, DI_ROW_NAME);
}

/***************************************************************************//**
 * This function is called to initiate factory reset. Factory reset may be
 * initiated by keeping one of the WSTK pushbuttons pressed during reboot.
 * Factory reset is also performed if it is requested by the provisioner
 * (event gecko_evt_mesh_node_reset_id).
 ******************************************************************************/
void initiate_factory_reset(void)
{
  log("factory reset\r\n");
  DI_Print("\n***\nFACTORY RESET\n***", DI_ROW_STATUS);

  /* if connection is open then close it before rebooting */
  if (conn_handle != 0xFF) {
    gecko_cmd_le_connection_close(conn_handle);
  }

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

/***************************************************************************//**
 * Main function.
 ******************************************************************************/
int main(void)
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();
  initVcomEnable();

  // Minimize advertisement latency by allowing the advertiser to always
  // interrupt the scanner.
  linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

  gecko_stack_init(&config);
  gecko_bgapi_class_dfu_init();
  gecko_bgapi_class_system_init();
  gecko_bgapi_class_le_gap_init();
  gecko_bgapi_class_le_connection_init();
  //gecko_bgapi_class_gatt_init();
  gecko_bgapi_class_gatt_server_init();
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();
  gecko_bgapi_class_test_init();
  //gecko_bgapi_class_sm_init();
  gecko_bgapi_class_mesh_node_init();
  //gecko_bgapi_class_mesh_prov_init();
  gecko_bgapi_class_mesh_proxy_init();
  gecko_bgapi_class_mesh_proxy_server_init();
  //gecko_bgapi_class_mesh_proxy_client_init();
  gecko_bgapi_class_mesh_generic_client_init();
  //gecko_bgapi_class_mesh_generic_server_init();
  //gecko_bgapi_class_mesh_vendor_model_init();
  //gecko_bgapi_class_mesh_health_client_init();
  //gecko_bgapi_class_mesh_health_server_init();
  //gecko_bgapi_class_mesh_test_init();
  gecko_bgapi_class_mesh_lpn_init();
  //gecko_bgapi_class_mesh_friend_init();
  gecko_bgapi_class_mesh_scene_client_init();

  // Initialize coexistence interface. Parameters are taken from HAL config.
  gecko_initCoexHAL();

  RETARGET_SerialInit();

  /* initialize LEDs and buttons. Note: some radio boards share the same GPIO for button & LED.
   * Initialization is done in this order so that default configuration will be "button" for those
   * radio boards with shared pins. led_init() is called later as needed to (re)initialize the LEDs
   * */
  led_init();
  button_init();

  // Display Interface initialization
  DI_Init();

#if defined(_SILICON_LABS_32B_SERIES_1_CONFIG_3)
  /* xG13 devices have two RTCCs, one for the stack and another for the application.
   * The clock for RTCC needs to be enabled in application code. In xG12 RTCC init
   * is handled by the stack */
  CMU_ClockEnable(cmuClock_RTCC, true);
#endif

  while (1) {
    struct gecko_cmd_packet *evt = gecko_wait_event();
    bool pass = mesh_bgapi_listener(evt);
    if (pass) {
      handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
    }
  }
}

/*******************************************************************************
 * Handling of stack events. Both Bluetooth LE and Bluetooth mesh events
 * are handled here.
 * @param[in] evt_id  Incoming event ID.
 * @param[in] evt     Pointer to incoming event.
 ******************************************************************************/
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  uint16_t result;
  char buf[30];

  if (NULL == evt) {
    return;
  }

  switch (evt_id) {
    case gecko_evt_system_boot_id:
      // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
      if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0 || GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0) {
        initiate_factory_reset();
      } else {
        struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

        set_device_name(&pAddr->address);

        // Initialize Mesh stack in Node operation mode, it will generate initialized event
        result = gecko_cmd_mesh_node_init()->result;
        if (result) {
          sprintf(buf, "init failed (0x%x)", result);
          DI_Print(buf, DI_ROW_STATUS);
        }
      }
      break;

    case gecko_evt_hardware_soft_timer_id:
      switch (evt->data.evt_hardware_soft_timer.handle) {
        case TIMER_ID_FACTORY_RESET:
          // reset the device to finish factory reset
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_RESTART:
          // restart timer expires, reset the device
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_PROVISIONING:
          // toggle LED to indicate the provisioning state
          LED_set_state(LED_STATE_PROV);
          break;

        case TIMER_ID_RETRANS_ONOFF:
          send_onoff_request(1);   // param 1 indicates that this is a retransmission
          // stop retransmission timer if it was the last attempt
          if (onoff_request_count == 0) {
            gecko_cmd_hardware_set_soft_timer(TIMER_STOP,
                                              TIMER_ID_RETRANS_ONOFF,
                                              0);
          }
          break;

        case TIMER_ID_RETRANS_LIGHTNESS:
          send_lightness_request(1);   // Retransmit lightness message
          // Stop retransmission timer if it was the last attempt
          if (lightness_request_count == 0) {
            gecko_cmd_hardware_set_soft_timer(TIMER_STOP,
                                              TIMER_ID_RETRANS_LIGHTNESS,
                                              0);
          }
          break;

        case TIMER_ID_RETRANS_CTL:
          send_ctl_request(1);   // Retransmit ctl message
          // Stop retransmission timer if it was the last attempt
          if (ctl_request_count == 0) {
            gecko_cmd_hardware_set_soft_timer(TIMER_STOP,
                                              TIMER_ID_RETRANS_CTL,
                                              0);
          }
          break;

        case TIMER_ID_RETRANS_SCENE:
          send_scene_recall_request(1);   // Retransmit scene message
          // Stop retransmission timer if it was the last attempt
          if (scene_request_count == 0) {
            gecko_cmd_hardware_set_soft_timer(TIMER_STOP,
                                              TIMER_ID_RETRANS_SCENE,
                                              0);
          }
          break;

        case TIMER_ID_NODE_CONFIGURED:
          if (!lpn_active) {
            log("try to initialize lpn...\r\n");
            lpn_init();
          }
          break;

        case TIMER_ID_FRIEND_FIND:
        {
          log("trying to find friend...\r\n");
          result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

          if (result != 0) {
            log("ret.code 0x%x\r\n", result);
          }
        }
        break;

        default:
          break;
      }

      break;

    case gecko_evt_mesh_node_initialized_id:
      log("node initialized\r\n");

      // Initialize generic client models
      result = gecko_cmd_mesh_generic_client_init()->result;
      if (result) {
        log("mesh_generic_client_init failed, code 0x%x\r\n", result);
      }

      // Initialize scene client model
      result = gecko_cmd_mesh_scene_client_init(0)->result;
      if (result) {
        log("mesh_scene_client_init failed, code 0x%x\r\n", result);
      }

      struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

      if (pData->provisioned) {
        log("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);

        _my_address = pData->address;
        _elem_index = 0;   // index of primary element is zero. This example has only one element.

        enable_button_interrupts();
        switch_node_init();

        // Initialize Low Power Node functionality
        lpn_init();

        DI_Print("provisioned", DI_ROW_STATUS);
      } else {
        log("node is unprovisioned\r\n");
        DI_Print("unprovisioned", DI_ROW_STATUS);

        log("starting unprovisioned beaconing...\r\n");
        gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
      }
      break;

    case gecko_evt_system_external_signal_id:
    {
      if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB0_SHORT_PRESS) {
        handle_short_press(0);
      }
      if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB1_SHORT_PRESS) {
        handle_short_press(1);
      }
      if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB0_MEDIUM_PRESS) {
        handle_medium_press(0);
      }
      if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB1_MEDIUM_PRESS) {
        handle_medium_press(1);
      }
      if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB0_LONG_PRESS) {
        handle_long_press(0);
      }
      if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB1_LONG_PRESS) {
        handle_long_press(1);
      }
      if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB0_VERY_LONG_PRESS) {
        handle_very_long_press(0);
      }
      if (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_PB1_VERY_LONG_PRESS) {
        handle_very_long_press(1);
      }
    }
    break;

    case gecko_evt_mesh_node_provisioning_started_id:
      log("Started provisioning\r\n");
      DI_Print("provisioning...", DI_ROW_STATUS);
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
      led_init(); /* shared GPIO pins used as LED output */
#endif
      // start timer for blinking LEDs to indicate which node is being provisioned
      gecko_cmd_hardware_set_soft_timer(32768 / 4, TIMER_ID_PROVISIONING, 0);
      break;

    case gecko_evt_mesh_node_provisioned_id:
      _elem_index = 0;   // index of primary element is zero. This example has only one element.
      switch_node_init();
      // try to initialize lpn after 30 seconds, if no configuration messages come
      result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(30000),
                                                 TIMER_ID_NODE_CONFIGURED,
                                                 1)->result;
      if (result) {
        log("timer failure?!  0x%x\r\n", result);
      }
      log("node provisioned, got address=%x\r\n", evt->data.evt_mesh_node_provisioned.address);
      // stop LED blinking when provisioning complete
      gecko_cmd_hardware_set_soft_timer(TIMER_STOP, TIMER_ID_PROVISIONING, 0);
      LED_set_state(LED_STATE_OFF);
      DI_Print("provisioned", DI_ROW_STATUS);

#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
      button_init(); /* shared GPIO pins used as button input */
#endif
      enable_button_interrupts();
      break;

    case gecko_evt_mesh_node_provisioning_failed_id:
      log("provisioning failed, code 0x%x\r\n", evt->data.evt_mesh_node_provisioning_failed.result);
      DI_Print("prov failed", DI_ROW_STATUS);
      /* start a one-shot timer that will trigger soft reset after small delay */
      gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
      break;

    case gecko_evt_mesh_node_key_added_id:
      log("got new %s key with index 0x%x\r\n",
          evt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
          evt->data.evt_mesh_node_key_added.index);
      // try to init lpn 5 seconds after adding key
      result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(5000),
                                                 TIMER_ID_NODE_CONFIGURED,
                                                 1)->result;
      if (result) {
        log("timer failure?!  0x%x\r\n", result);
      }
      break;

    case gecko_evt_mesh_node_model_config_changed_id:
      log("model config changed\r\n");
      // try to init lpn 5 seconds after configuration change
      result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(5000),
                                                 TIMER_ID_NODE_CONFIGURED,
                                                 1)->result;
      if (result) {
        log("timer failure?!  0x%x\r\n", result);
      }
      break;

    case gecko_evt_mesh_node_config_set_id:
      log("model config set\r\n");
      // try to init lpn 5 seconds after configuration set
      result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(5000),
                                                 TIMER_ID_NODE_CONFIGURED,
                                                 1)->result;
      if (result) {
        log("timer failure?!  0x%x\r\n", result);
      }
      break;

    case gecko_evt_le_connection_opened_id:
      log("evt:gecko_evt_le_connection_opened_id\r\n");
      num_connections++;
      conn_handle = evt->data.evt_le_connection_opened.connection;
      DI_Print("connected", DI_ROW_CONNECTION);
      // turn off lpn feature after GATT connection is opened
      lpn_deinit();
      break;

    case gecko_evt_le_connection_closed_id:
      /* Check if need to boot to dfu mode */
      if (boot_to_dfu) {
        /* Enter to DFU OTA mode */
        gecko_cmd_system_reset(2);
      }

      log("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
      conn_handle = 0xFF;
      if (num_connections > 0) {
        if (--num_connections == 0) {
          DI_Print("", DI_ROW_CONNECTION);
          // initialize lpn when there is no active connection
          lpn_init();
        }
      }
      break;

    case gecko_evt_mesh_node_reset_id:
      log("evt gecko_evt_mesh_node_reset_id\r\n");
      initiate_factory_reset();
      break;

    case gecko_evt_le_connection_parameters_id:
      log("connection params: interval %d, timeout %d\r\n",
          evt->data.evt_le_connection_parameters.interval,
          evt->data.evt_le_connection_parameters.timeout);
      break;

    case gecko_evt_le_gap_adv_timeout_id:
      // these events silently discarded
      break;

    case gecko_evt_gatt_server_user_write_request_id:
      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
        /* Set flag to enter to OTA mode */
        boot_to_dfu = 1;
        /* Send response to Write Request */
        gecko_cmd_gatt_server_send_user_write_response(
          evt->data.evt_gatt_server_user_write_request.connection,
          gattdb_ota_control,
          bg_err_success);

        /* Close connection to enter to DFU OTA mode */
        gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
      }
      break;

    case gecko_evt_mesh_lpn_friendship_established_id:
      log("friendship established\r\n");
      DI_Print("LPN with friend", DI_ROW_LPN);
      break;

    case gecko_evt_mesh_lpn_friendship_failed_id:
      log("friendship failed\r\n");
      DI_Print("no friend", DI_ROW_LPN);
      // try again in 2 seconds
      result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
                                                 TIMER_ID_FRIEND_FIND,
                                                 1)->result;
      if (result) {
        log("timer failure?!  0x%x\r\n", result);
      }
      break;

    case gecko_evt_mesh_lpn_friendship_terminated_id:
      log("friendship terminated\r\n");
      DI_Print("friend lost", DI_ROW_LPN);
      if (num_connections == 0) {
        // try again in 2 seconds
        result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
                                                   TIMER_ID_FRIEND_FIND,
                                                   1)->result;
        if (result) {
          log("timer failure?!  0x%x\r\n", result);
        }
      }
      break;

    default:
      //log("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
      break;
  }
}
/******************************************************************
 * Added for LC
 * ***************************************************************/
uint16_t get_primary_elem_addr(void)
{
  return _my_address;
}

