#pragma once

#include "opendbc/safety/declarations.h"

// Stock longitudinal
#define TOYOTA_BASE_TX_MSGS \
  {0x191, 0, 8, .check_relay = true}, {0x412, 0, 8, .check_relay = true}, {0x1D2, 0, 8, .check_relay = false},  /* LKAS + LTA + PCM cancel cmd */  \

#define TOYOTA_COMMON_TX_MSGS \
  TOYOTA_BASE_TX_MSGS \
  {0x2E4, 0, 5, .check_relay = true}, \
  {0x343, 0, 8, .check_relay = false},  /* ACC cancel cmd */  \

#define TOYOTA_COMMON_SECOC_TX_MSGS \
  TOYOTA_BASE_TX_MSGS \
  {0x2E4, 0, 8, .check_relay = true}, {0x131, 0, 8, .check_relay = true}, \
  {0x343, 0, 8, .check_relay = false},  /* ACC cancel cmd */ \

// LionDriver TSR-705, decision D-1: the reference build (LIONDRIVER_REFERENCE, 2020 Corolla LE TSS2 with camera ACC)
// never sends PCS_HUD, so it does not accept it. Other builds keep it for radar-ACC cars with alpha longitudinal,
// which disable the radar and send their own PCS_HUD
#ifdef LIONDRIVER_REFERENCE
#define TOYOTA_PCS_HUD_TX_MSG
#else
#define TOYOTA_PCS_HUD_TX_MSG {0x411, 0, 8, .check_relay = false},
#endif

#define TOYOTA_COMMON_LONG_TX_MSGS \
  TOYOTA_COMMON_TX_MSGS \
  /* PRE_COLLISION, zero payload only (see toyota_tx_hook). LionDriver TSR-705: PRE_COLLISION_2 (0x344), */ \
  /* the DSU bus-0 messages 0x2E6, 0x2E7, 0x33E, 0x365, 0x366, 0x4CB and the DSU bus-1 set are not allowed; */ \
  /* no openpilot code sends them, and 0x344 carries PCS brake triggers */ \
  {0x283, 0, 7, .check_relay = false}, \
  /* PCS_HUD, not in the LionDriver reference build */ \
  TOYOTA_PCS_HUD_TX_MSG \
  /* radar diagnostic address */       \
  {0x750, 0, 8, .check_relay = false}, \
  /* ACC */                            \
  {0x343, 0, 8, .check_relay = true},  \

#define TOYOTA_COMMON_SECOC_LONG_TX_MSGS \
  TOYOTA_COMMON_SECOC_TX_MSGS \
  {0x343, 0, 8, .check_relay = true}, \
  {0x183, 0, 8, .check_relay = true},  /* ACC_CONTROL_2 */ \

#define TOYOTA_COMMON_RX_CHECKS(lta)                                                                                                       \
  {.msg = {{ 0xaa, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},      \
  {.msg = {{0x260, 0, 8, 50U, .ignore_counter = true, .ignore_quality_flag=!(lta)}, { 0 }, { 0 }}},  \

// LionDriver TSR-110, TSR-401: the reference build also supervises EPS_STATUS (0x262) and PCM_CRUISE_2 (0x1D3) for
// presence, checksum and timeout on the reference car's RX set (non-SecOC, BRAKE_MODULE brake; reference_lock fixes the
// parameter), as inputs of the planned EPS-fault (TSR-110) and brake/cruise cross-checks (TSR-405).
// The nominal rates are the minimum rates openpilot's CAN parser required for these messages (25 Hz, 33 Hz); they are
// to be confirmed against measured rates on the reference vehicle (WP-S-05 OI-7). SPEED (0xB4) is not added: no
// nominal rate is known yet, and a rate set too high would block engagement permanently.
#ifdef LIONDRIVER_REFERENCE
#define TOYOTA_REFERENCE_RX_CHECKS                                                                                                          \
  {.msg = {{0x262, 0, 8, 25U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            \
  {.msg = {{0x1D3, 0, 8, 33U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            \

#else
#define TOYOTA_REFERENCE_RX_CHECKS
#endif

#define TOYOTA_RX_CHECKS(lta)                                                                                                               \
  TOYOTA_COMMON_RX_CHECKS(lta)                                                                                                              \
  TOYOTA_REFERENCE_RX_CHECKS                                                                                                                \
  {.msg = {{0x1D2, 0, 8, 33U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            \
  {.msg = {{0x226, 0, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  { 0 }, { 0 }}},  \

#define TOYOTA_ALT_BRAKE_RX_CHECKS(lta)                                                                                                    \
  TOYOTA_COMMON_RX_CHECKS(lta)                                                                                                             \
  {.msg = {{0x1D2, 0, 8, 33U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                           \
  {.msg = {{0x224, 0, 8, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define TOYOTA_SECOC_RX_CHECKS                                                                                                             \
  TOYOTA_COMMON_RX_CHECKS(false)                                                                                                           \
  {.msg = {{0x176, 0, 8, 32U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                           \
  {.msg = {{0x116, 0, 8, 42U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0x101, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

// LionDriver TSR-401: RX timeout threshold in nominal message periods
#define TOYOTA_RX_TIMEOUT_PERIODS 5U

static bool toyota_secoc = false;
static bool toyota_alt_brake = false;
static bool toyota_stock_longitudinal = false;
static bool toyota_lta = false;
static int toyota_dbc_eps_torque_factor = 100;   // conversion factor for STEER_TORQUE_EPS in %: see dbc file

#ifdef LIONDRIVER_REFERENCE
// LionDriver TSR-110 / SWSR-110: lateral actuation is refused while EPS_STATUS.LKA_STATE reports a fault or
// unavailability. The code set is the one openpilot's host already treats as a steering fault (TEMP_STEER_FAULTS and
// PERM_STEER_FAULTS in opendbc/car/toyota/carstate.py), so the envelope and the host stop lateral control on the same
// codes; it is reverse-engineered and to be confirmed for the reference EPS firmware. Authority cannot be granted before
// a valid EPS_STATUS is received (RX check, TSR-306), and every valid frame updates the state. The state is kept in
// safety_eps_fault, which the panda reports to the host (reset on every safety mode change).

static bool toyota_lka_state_is_fault(int lka_state) {
  bool fault;
  switch (lka_state) {
    case 0:   // initializing / catch-all
    case 3:   // LKA message missing, unavailable
    case 9:   // temporary fault 2
    case 11:  // LKA message missing, unavailable 2
    case 17:  // permanent fault (prolonged high driver torque)
    case 21:
    case 25:  // temporary fault
      fault = true;
      break;
    default:
      fault = false;
      break;
  }
  return fault;
}

// LionDriver TSR-405(c) / SWSR-405(c): PCM_CRUISE.CRUISE_ACTIVE (0x1D2) set while PCM_CRUISE_2.MAIN_ON (0x1D3) is 0 is
// not a valid state. Cruise becoming active while main is off is refused at once; main going off while cruise is
// active (the driver switching main off, where the two messages can disagree for a frame or two) revokes authority
// once the disagreement has lasted longer than the tolerance. Not used for alt-brake cars, whose RX set has no 0x1D3.
#define TOYOTA_CRUISE_MAIN_TOLERANCE_US 150000U  // about 5 periods of the 33 Hz messages; detection <= 0.2 s (TSR-405)
static bool toyota_main_on = false;
static bool toyota_cruise_main_mismatch = false;
static uint32_t toyota_cruise_main_mismatch_start = 0U;

static void toyota_cruise_main_check(bool from_pcm_cruise) {
  bool mismatch = cruise_engaged_prev && !toyota_main_on;
  if (!mismatch) {
    toyota_cruise_main_mismatch = false;
  } else if (!toyota_cruise_main_mismatch) {
    toyota_cruise_main_mismatch = true;
    toyota_cruise_main_mismatch_start = microsecond_timer_get();
    if (from_pcm_cruise) {
      // cruise became active while main is off: never a valid engagement
      controls_allowed = false;
    }
  } else {
    uint32_t elapsed = safety_get_ts_elapsed(microsecond_timer_get(), toyota_cruise_main_mismatch_start);
    if (elapsed > TOYOTA_CRUISE_MAIN_TOLERANCE_US) {
      controls_allowed = false;
    }
  }
}
#endif

static uint32_t toyota_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint8_t checksum = (uint8_t)(msg->addr) + (uint8_t)((unsigned int)(msg->addr) >> 8U) + (uint8_t)(len);
  for (int i = 0; i < (len - 1); i++) {
    checksum += (uint8_t)msg->data[i];
  }
  return checksum;
}

static uint32_t toyota_get_checksum(const CANPacket_t *msg) {
  int checksum_byte = GET_LEN(msg) - 1U;
  return (uint8_t)(msg->data[checksum_byte]);
}

static bool toyota_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == 0x260U) {
    valid = !GET_BIT(msg, 3U);  // STEER_TORQUE_SENSOR.STEER_ANGLE_INITIALIZING
  }
  if (msg->addr == 0xaaU) {  // WHEEL_SPEEDS
    // each wheel speed is 1-bit fault + 15-bit speed
    valid = true;
    for (uint8_t i = 0U; i < 4U; i += 1U) {
      if (GET_BIT(msg, (i * 16U) + 7U)) {  // WHEEL_SPEED_xx_FAULT
        valid = false;
        break;
      }
    }
  }
  return valid;
}

static void toyota_rx_hook(const CANPacket_t *msg) {
  // get eps motor torque (0.66 factor in dbc)
  if (msg_matches(msg, 0x260U, 0U)) {
    int torque_meas_new = (msg->data[5] << 8) | msg->data[6];
    torque_meas_new = to_signed(torque_meas_new, 16);

    // scale by dbc_factor
    torque_meas_new = (torque_meas_new * toyota_dbc_eps_torque_factor) / 100;

    // update array of sample
    update_sample(&torque_meas, torque_meas_new);

    // increase torque_meas by 1 to be conservative on rounding
    torque_meas.min--;
    torque_meas.max++;

    // driver torque for angle limiting
    int torque_driver_new = (msg->data[1] << 8) | msg->data[2];
    torque_driver_new = to_signed(torque_driver_new, 16);
    update_sample(&torque_driver, torque_driver_new);

    // LTA request angle should match current angle while inactive, clipped to max accepted angle.
    // note that angle can be relative to init angle on some TSS2 platforms, LTA has the same offset
    bool steer_angle_initializing = GET_BIT(msg, 3U);
    if (!steer_angle_initializing) {
      int angle_meas_new = (msg->data[3] << 8U) | msg->data[4];
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);
    }
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  // exit controls on rising edge of gas press, if not alternative experience
  // exit controls on rising edge of brake press
  if (toyota_secoc) {
    if (msg_matches(msg, 0x176U, 0U)) {
      bool cruise_engaged = GET_BIT(msg, 5U);  // PCM_CRUISE.CRUISE_ACTIVE
      pcm_cruise_check(cruise_engaged);
    }
    if (msg_matches(msg, 0x116U, 0U)) {
      gas_pressed = msg->data[1] != 0U;  // GAS_PEDAL.GAS_PEDAL_USER
    }
    if (msg_matches(msg, 0x101U, 0U)) {
      brake_pressed = GET_BIT(msg, 3U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_rav4_prime_generated.dbc)
    }
  } else {
    if (msg_matches(msg, 0x1D2U, 0U)) {
      bool cruise_engaged = GET_BIT(msg, 5U);  // PCM_CRUISE.CRUISE_ACTIVE
      pcm_cruise_check(cruise_engaged);
      gas_pressed = !GET_BIT(msg, 4U);  // PCM_CRUISE.GAS_RELEASED
    }
    if (!toyota_alt_brake && msg_matches(msg, 0x226U, 0U)) {
      brake_pressed = GET_BIT(msg, 37U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_nodsu_pt_generated.dbc)
    }
    if (toyota_alt_brake && msg_matches(msg, 0x224U, 0U)) {
      brake_pressed = GET_BIT(msg, 5U);  // BRAKE_MODULE.BRAKE_PRESSED (toyota_new_mc_pt_generated.dbc)
    }
#ifdef LIONDRIVER_REFERENCE
    if (msg_matches(msg, 0x262U, 0U, 8U)) {
      int lka_state = (msg->data[3] >> 1U) & 0x7FU;  // EPS_STATUS.LKA_STATE (toyota_nodsu_pt_generated.dbc)
      safety_eps_fault = toyota_lka_state_is_fault(lka_state);
    }
    if (!toyota_alt_brake) {
      if (msg_matches(msg, 0x1D3U, 0U, 8U)) {
        toyota_main_on = GET_BIT(msg, 15U);  // PCM_CRUISE_2.MAIN_ON
        toyota_cruise_main_check(false);
      }
      if (msg_matches(msg, 0x1D2U, 0U)) {
        toyota_cruise_main_check(true);
      }
    }
#endif
  }

  // sample speed
  if (msg_matches(msg, 0xaaU, 0U)) {
    int speed = 0;
    // sum 4 wheel speeds. conversion: raw * 0.01 - 67.67
    for (uint8_t i = 0U; i < 8U; i += 2U) {
      int wheel_speed = ((msg->data[i] & 0x7FU) << 8U) | msg->data[(i + 1U)];
      speed += wheel_speed - 6767;
    }
    // check that all wheel speeds are at zero value
    vehicle_moving = speed != 0;

    UPDATE_VEHICLE_SPEED(speed / 4.0 * 0.01 * KPH_TO_MS);
  }
}

static bool toyota_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits TOYOTA_TORQUE_STEERING_LIMITS = {
    .max_torque = 1500,
    .max_rate_up = 15,          // ramp up slow
    .max_rate_down = 25,        // ramp down fast
    .max_torque_error = 350,    // max torque cmd in excess of motor torque
    .max_rt_delta = 450,        // the real time limit is 1800/sec, a 20% buffer
    .type = TorqueMotorLimited,

    // the EPS faults when the steering angle rate is above a certain threshold for too long. to prevent this,
    // we allow setting STEER_REQUEST bit to 0 while maintaining the requested torque value for a single frame
    .min_valid_request_frames = 17,
    .max_invalid_request_frames = 1,
    .min_valid_request_rt_interval = 162000,  // 162ms; a ~10% buffer on cutting every 18 frames
    .has_steer_req_tolerance = true,
  };

  static const AngleSteeringLimits TOYOTA_ANGLE_STEERING_LIMITS = {
    // LTA angle limits
    // factor for STEER_TORQUE_SENSOR->STEER_ANGLE and STEERING_LTA->STEER_ANGLE_CMD (1 / 0.0573)
    .max_angle = 1657,  // EPS only accepts up to 94.9461
    .angle_deg_to_can = 17.452007,
    .angle_rate_up_lookup = {
      {5., 25., 25.},
      {0.3, 0.15, 0.15}
    },
    .angle_rate_down_lookup = {
      {5., 25., 25.},
      {0.36, 0.26, 0.26}
    },
  };

  const int TOYOTA_LTA_MAX_MEAS_TORQUE = 1500;
  const int TOYOTA_LTA_MAX_DRIVER_TORQUE = 150;

  // longitudinal limits
  const LongitudinalLimits TOYOTA_LONG_LIMITS = {
    .max_accel = 2000,   // 2.0 m/s2
    .min_accel = -3500,  // -3.5 m/s2
  };

  bool tx = true;

  // Check if msg is sent on BUS 0
  // ACCEL: safety check on byte 1-2
  if (msg_matches(msg, 0x343U, 0U)) {
    int desired_accel = (msg->data[0] << 8) | msg->data[1];
    desired_accel = to_signed(desired_accel, 16);

    bool violation = false;
    if (toyota_secoc) {
      // SecOC cars move accel to 0x183. Only allow inactive accel on 0x343 to match stock behavior
      violation = desired_accel != TOYOTA_LONG_LIMITS.inactive_accel;
    }
    violation |= longitudinal_accel_checks(desired_accel, TOYOTA_LONG_LIMITS);

    // only ACC messages that cancel are allowed when openpilot is not controlling longitudinal
    if (toyota_stock_longitudinal) {
      bool cancel_req = GET_BIT(msg, 24U);
      if (!cancel_req) {
        violation = true;
      }
      if (desired_accel != TOYOTA_LONG_LIMITS.inactive_accel) {
        violation = true;
      }
    }

    if (violation) {
      tx = false;
    }
  }

  if (msg_matches(msg, 0x183U, 0U)) {
    int desired_accel = (msg->data[0] << 8) | msg->data[1];
    desired_accel = to_signed(desired_accel, 16);

    tx = !longitudinal_accel_checks(desired_accel, TOYOTA_LONG_LIMITS);
  }

  // AEB: block all actuation. only used when DSU is unplugged
  if (msg_matches(msg, 0x283U, 0U)) {
    // only allow the checksum, which is the last byte
    bool block = GET_BYTES_64_LE(msg, 0, 6) != 0U;
    if (block) {
      tx = false;
    }
  }

  // STEERING_LTA angle steering check
  if (msg_matches(msg, 0x191U, 0U)) {
    // check the STEER_REQUEST, STEER_REQUEST_2, TORQUE_WIND_DOWN, STEER_ANGLE_CMD signals
    bool lta_request = GET_BIT(msg, 0U);
    bool lta_request2 = GET_BIT(msg, 25U);
    int torque_wind_down = msg->data[5];
    int lta_angle = (msg->data[1] << 8) | msg->data[2];
    lta_angle = to_signed(lta_angle, 16);

    bool steer_control_enabled = lta_request || lta_request2;
    if (!toyota_lta) {
      // using torque (LKA), block LTA msgs with actuation requests
      if (steer_control_enabled || (lta_angle != 0) || (torque_wind_down != 0)) {
        tx = false;
      }
    } else {
      // check angle rate limits and inactive angle
      if (steer_angle_cmd_checks(lta_angle, steer_control_enabled, TOYOTA_ANGLE_STEERING_LIMITS)) {
        tx = false;
      }

      if (lta_request != lta_request2) {
        tx = false;
      }

      // TORQUE_WIND_DOWN is gated on steer request
      if (!steer_control_enabled && (torque_wind_down != 0)) {
        tx = false;
      }

      // TORQUE_WIND_DOWN can only be no or full torque
      if ((torque_wind_down != 0) && (torque_wind_down != 100)) {
        tx = false;
      }

      // check if we should wind down torque
      int driver_torque = SAFETY_MIN(SAFETY_ABS(torque_driver.min), SAFETY_ABS(torque_driver.max));
      if ((driver_torque > TOYOTA_LTA_MAX_DRIVER_TORQUE) && (torque_wind_down != 0)) {
        tx = false;
      }

      int eps_torque = SAFETY_MIN(SAFETY_ABS(torque_meas.min), SAFETY_ABS(torque_meas.max));
      if ((eps_torque > TOYOTA_LTA_MAX_MEAS_TORQUE) && (torque_wind_down != 0)) {
        tx = false;
      }
    }
  }

  // STEERING_LTA_2 angle steering check (SecOC)
  if (toyota_secoc && msg_matches(msg, 0x131U, 0U)) {
    // SecOC cars block any form of LTA actuation for now
    bool lta_request = GET_BIT(msg, 3U);  // STEERING_LTA_2.STEER_REQUEST
    bool lta_request2 = GET_BIT(msg, 0U);  // STEERING_LTA_2.STEER_REQUEST_2
    int lta_angle_msb = msg->data[2];  // STEERING_LTA_2.STEER_ANGLE_CMD (MSB)
    int lta_angle_lsb = msg->data[3];  // STEERING_LTA_2.STEER_ANGLE_CMD (LSB)

    bool actuation = lta_request || lta_request2 || (lta_angle_msb != 0) || (lta_angle_lsb != 0);
    if (actuation) {
      tx = false;
    }
  }

  // STEER: safety check on bytes 2-3
  if (msg_matches(msg, 0x2E4U, 0U)) {
    int desired_torque = (msg->data[1] << 8) | msg->data[2];
    desired_torque = to_signed(desired_torque, 16);
    bool steer_req = GET_BIT(msg, 0U);
    // When using LTA (angle control), assert no actuation on LKA message
    if (!toyota_lta) {
#ifdef LIONDRIVER_REFERENCE
      // LionDriver TSR-110: no torque and no steer request while the EPS reports a fault (checked before the limit
      // checks so that a refused frame does not update their state)
      if (safety_eps_fault && ((desired_torque != 0) || steer_req)) {
        tx = false;
      } else {
        if (steer_torque_cmd_checks(desired_torque, steer_req, TOYOTA_TORQUE_STEERING_LIMITS)) {
          tx = false;
        }
      }
#else
      if (steer_torque_cmd_checks(desired_torque, steer_req, TOYOTA_TORQUE_STEERING_LIMITS)) {
        tx = false;
      }
#endif
    } else {
      if ((desired_torque != 0) || steer_req) {
        tx = false;
      }
    }
  }

  // UDS: Only tester present ("\x0F\x02\x3E\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (msg->addr == 0x750U) {
    // this address is sub-addressed. only allow tester present to radar (0xF)
    bool invalid_uds_msg = GET_BYTES_64_LE(msg, 0, 8) != 0x00000000003E020FULL;
    if (invalid_uds_msg) {
      tx = false;
    }
  }

  return tx;
}

static safety_config toyota_init(uint16_t param) {
  static const CanMsg TOYOTA_TX_MSGS[] = {
    TOYOTA_COMMON_TX_MSGS
  };

  static const CanMsg TOYOTA_SECOC_TX_MSGS[] = {
    TOYOTA_COMMON_SECOC_TX_MSGS
  };

  static const CanMsg TOYOTA_LONG_TX_MSGS[] = {
    TOYOTA_COMMON_LONG_TX_MSGS
  };

  static const CanMsg TOYOTA_SECOC_LONG_TX_MSGS[] = {
    TOYOTA_COMMON_SECOC_LONG_TX_MSGS
  };

  // safety param flags
  // first byte is for EPS factor, second is for flags
  const uint32_t TOYOTA_PARAM_OFFSET = 8U;
  const uint32_t TOYOTA_EPS_FACTOR = (1UL << TOYOTA_PARAM_OFFSET) - 1U;
  const uint32_t TOYOTA_PARAM_ALT_BRAKE = 1UL << TOYOTA_PARAM_OFFSET;
  const uint32_t TOYOTA_PARAM_STOCK_LONGITUDINAL = 2UL << TOYOTA_PARAM_OFFSET;
  const uint32_t TOYOTA_PARAM_LTA = 4UL << TOYOTA_PARAM_OFFSET;

#ifdef ALLOW_DEBUG
  const uint32_t TOYOTA_PARAM_SECOC = 8UL << TOYOTA_PARAM_OFFSET;
  toyota_secoc = GET_FLAG(param, TOYOTA_PARAM_SECOC);
#endif

  toyota_alt_brake = GET_FLAG(param, TOYOTA_PARAM_ALT_BRAKE);
  toyota_stock_longitudinal = GET_FLAG(param, TOYOTA_PARAM_STOCK_LONGITUDINAL);
  toyota_lta = GET_FLAG(param, TOYOTA_PARAM_LTA);
  toyota_dbc_eps_torque_factor = param & TOYOTA_EPS_FACTOR;
#ifdef LIONDRIVER_REFERENCE
  toyota_main_on = false;
  toyota_cruise_main_mismatch = false;
  toyota_cruise_main_mismatch_start = 0U;
#endif

  safety_config ret;
  if (toyota_secoc) {
    if (toyota_stock_longitudinal) {
      SET_TX_MSGS(TOYOTA_SECOC_TX_MSGS, ret);
    } else {
      SET_TX_MSGS(TOYOTA_SECOC_LONG_TX_MSGS, ret);
    }
  } else {
    if (toyota_stock_longitudinal) {
      SET_TX_MSGS(TOYOTA_TX_MSGS, ret);
    } else {
      SET_TX_MSGS(TOYOTA_LONG_TX_MSGS, ret);
    }
  }

  if (toyota_secoc) {
    static RxCheck toyota_secoc_rx_checks[] = {
      TOYOTA_SECOC_RX_CHECKS
    };

    SET_RX_CHECKS(toyota_secoc_rx_checks, ret);
  } else if (toyota_lta) {
    // Check the quality flag for angle measurement when using LTA, since it's not set on TSS-P cars
    static RxCheck toyota_lta_rx_checks[] = {
      TOYOTA_RX_CHECKS(true)
    };

    SET_RX_CHECKS(toyota_lta_rx_checks, ret);
  } else {
    static RxCheck toyota_lka_rx_checks[] = {
      TOYOTA_RX_CHECKS(false)
    };
    static RxCheck toyota_lka_alt_brake_rx_checks[] = {
      TOYOTA_ALT_BRAKE_RX_CHECKS(false)
    };

    if (!toyota_alt_brake) {
      SET_RX_CHECKS(toyota_lka_rx_checks, ret);
    } else {
      SET_RX_CHECKS(toyota_lka_alt_brake_rx_checks, ret);
    }
  }

  // LionDriver TSR-401: an RX check lags after 5 missed periods (0x1D2 at 33 Hz: 0.15 s, plus one 10 ms safety task period)
  ret.rx_timeout_periods = TOYOTA_RX_TIMEOUT_PERIODS;

#ifdef LIONDRIVER_REFERENCE
  // LionDriver TSR-518 (DN-01): TX rate budgets for the non-SecOC configurations (the reference car). Nominal rates from
  // opendbc/car/toyota/carcontroller.py; burst covers on-change frames and host catch-up after a short stall
  if (!toyota_secoc) {
    static TxRateCheck toyota_tx_rate_checks[] = {
      {.addr = 0x2E4U, .bus = 0U, .frequency = 100U, .burst = 5U},  // STEERING_LKA, every control cycle
      {.addr = 0x191U, .bus = 0U, .frequency = 50U, .burst = 3U},   // STEERING_LTA
      {.addr = 0x343U, .bus = 0U, .frequency = 33U, .burst = 3U},   // ACC_CONTROL
      {.addr = 0x412U, .bus = 0U, .frequency = 5U, .burst = 4U},    // LKAS_HUD, 5 Hz and on change
      {.addr = 0x1D2U, .bus = 0U, .frequency = 1U, .burst = 2U},    // PCM cancel, not sent by the reference car
      {.addr = 0x283U, .bus = 0U, .frequency = 1U, .burst = 2U},    // PRE_COLLISION, zero payload, not sent
      {.addr = 0x750U, .bus = 0U, .frequency = 5U, .burst = 2U},    // radar tester present, DISABLE_RADAR cars only
    };
    SET_TX_RATE_CHECKS(toyota_tx_rate_checks, ret);
  }
#endif

  return ret;
}

const safety_hooks toyota_hooks = {
  .init = toyota_init,
  .rx = toyota_rx_hook,
  .tx = toyota_tx_hook,
  .get_checksum = toyota_get_checksum,
  .compute_checksum = toyota_compute_checksum,
  .get_quality_flag_valid = toyota_get_quality_flag_valid,
};
