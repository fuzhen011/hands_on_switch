/* Includes *********************************************************** */
#include "lc_sensor.h"
#include "gpiointerrupt.h"

#include "native_gecko.h"
#include "mesh_sensor.h"

#include "si1133.h"

#undef SUB_MODULE_NAME
#define SUB_MODULE_NAME "LC_Client"
#include "log.h"

/* Defines  *********************************************************** */
#define REPEATED 0
#define SINGLE_TIME 1

#define LUX_TO_STATE(x) (uint32_t)((x) * 100)

#define SENSOR_ELEMENT     0 ///< Sensor model located in primary element
#define NO_FLAGS           0 ///< No flags used for message
#define VALUE_IS_NOT_KNOWN  (0xFFFF) ///< People count value is not known
/* Global Variables *************************************************** */
typedef struct ambient_light{
  float lux;
  float uv; /* Not used in this example */
}ambient_light_t;

typedef struct lc_sensors{
  ambient_light_t amb_light;
  count16_t people_count;
}lc_sensors_t;

extern uint16_t get_primary_elem_addr(void);

/* Static Variables *************************************************** */
static lc_sensors_t sensor_states = { 0 };

/* Static Functions Declaractions ************************************* */
static void button_init(void);
static void enable_button_interrupts(void);
static void people_count_init(void);
static void ambient_light_init(void);

void sensors_init(void)
{
  people_count_init();
  ambient_light_init();
}

static void people_count_init(void)
{
  button_init();
  enable_button_interrupts();
}

static void people_count_change(bool inc)
{
  if (inc) {
    if (sensor_states.people_count != VALUE_IS_NOT_KNOWN - 1) {
      sensor_states.people_count++;
    }
  } else {
    if (sensor_states.people_count != 0) {
      sensor_states.people_count--;
    }
  }

  LOGD("PC = %u\n", sensor_states.people_count);

  if (get_primary_elem_addr() == 0) {
    /* If it's not in a network yet, return */
    return;
  }
  /* Send the people count after 50ms delay to because the GPIO filter doesn't
   * work as expected */
  BG_AST(gecko_cmd_hardware_set_soft_timer(TICKS_PER_MS(50),
                                           PEOPLE_COUNT_TIMER_ID,
                                           SINGLE_TIME)->result);
}

static void send_people_count(void)
{
  uint8_t len = 0, sensor_raw[20];

  len += mesh_sensor_data_to_buf(PEOPLE_COUNT,
                                 &sensor_raw[len],
                                 (uint8_t*)&sensor_states.people_count);

  /*
   * TODO
   *
   * Send the sensor status, which is already filled into sensor_raw buffer, to
   * the LC server.
   *
   * You could either:
   *   - Publish the value to the group address that the LC server subscribes
   *   from.
   *   - Send unicast message to the address of the element where the LC server
   *   resides.
   *
   * Note: Remember to check the return value to know whether the sending is
   * succeeded or not.
   */
#if 0
  BG_AST(gecko_cmd_mesh_sensor_server_send_status(SENSOR_ELEMENT,
                                                  get_primary_elem_addr() + 1,
                                                  0,
                                                  NO_FLAGS,
                                                  len, sensor_data)->result);
#endif

  BG_AST(gecko_cmd_hardware_set_soft_timer(PEOPLE_COUNT_UPDATE_INTERVAL,
                                           PEOPLE_COUNT_TIMER_ID,
                                           SINGLE_TIME)->result);
}

static void ambient_light_init(void)
{
  SI1133_init();

  BG_AST(gecko_cmd_hardware_set_soft_timer(AMBIENT_LIGHT_READ_INTERVAL,
                                           AMBIENT_LIGHT_SENSOR_TIMER_ID,
                                           REPEATED)->result);
}

static void ambient_light_send(void)
{
  illuminance_t lux;
  uint8_t len = 0, sensor_raw[20];

  if (get_primary_elem_addr() == 0) {
    /* If it's not in a network yet, return */
    return;
  }

  lux = LUX_TO_STATE(sensor_states.amb_light.lux);

  len += mesh_sensor_data_to_buf(PRESENT_AMBIENT_LIGHT_LEVEL,
                                 &sensor_raw[len],
                                 (uint8_t*)&lux);

  /*
   * TODO
   *
   * Send the sensor status, which is already filled into sensor_raw buffer, to
   * the LC server.
   *
   * You could either:
   *   - Publish the value to the group address that the LC server subscribes
   *   from.
   *   - Send unicast message to the address of the element where the LC server
   *   resides.
   *
   * Note: Remember to check the return value to know whether the sending is
   * succeeded or not.
   */
#if 0
  BG_AST(gecko_cmd_mesh_sensor_server_send_status(SENSOR_ELEMENT,
                                                  address,
                                                  0,
                                                  NO_FLAGS,
                                                  len, sensor_data)->result);
#endif
}

/*******************************************************************************
 * Button initialization. Configure pushbuttons PB0, PB1 as inputs.
 ******************************************************************************/
static void button_init(void)
{
  // configure pushbutton PB0 and PB1 as inputs, with pull-up enabled
  GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInputPullFilter, 1);
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
static void button_interrupt(uint8_t pin)
{
  if (pin == BSP_BUTTON0_PIN) {
    if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 1) {
      gecko_external_signal(EXT_SIGNAL_INCREMENT);
    }
  } else if (pin == BSP_BUTTON1_PIN) {
    if (GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 1) {
      gecko_external_signal(EXT_SIGNAL_DECREMENT);
    }
  }
}

/*******************************************************************************
 * Enable button interrupts for PB0, PB1. Both GPIOs are configured to trigger
 * an interrupt on the rising edge (button released).
 ******************************************************************************/
static void enable_button_interrupts(void)
{
  GPIOINT_Init();

  /* configure interrupt for PB0 and PB1, rising edges */
  GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN,
                    true, false, true);
  GPIO_ExtIntConfig(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, BSP_BUTTON1_PIN,
                    true, false, true);

  /* register the callback function that is invoked when interrupt occurs */
  GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, button_interrupt);
  GPIOINT_CallbackRegister(BSP_BUTTON1_PIN, button_interrupt);
}

void button_press_handler(uint32_t which)
{
  if (which == EXT_SIGNAL_DECREMENT) {
    people_count_change(0);
  } else if (which == EXT_SIGNAL_INCREMENT) {
    people_count_change(1);
  }
}
