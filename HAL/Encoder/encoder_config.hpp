#ifndef ENCODER_CONFIG_HPP
#define ENCODER_CONFIG_HPP

/* ================================================================
 * Encoder GPIO Pin Assignments — 4 motors with encoders
 *
 * Layout:
 *   ENCODER1 → Motor FL (front-left)
 *   ENCODER2 → Motor RL (rear-left)
 *   ENCODER3 → Motor FR (front-right)
 *   ENCODER4 → Motor RR (rear-right)
 * ================================================================ */

/* ── Encoder 1 — Motor FL (front-left) ───────────────────────── */
#define ENCODER1_PIN_A 18
#define ENCODER1_PIN_B 19

/* ── Encoder 2 — Motor RL (rear-left) ────────────────────────── */
#define ENCODER2_PIN_A 20
#define ENCODER2_PIN_B 21

/* ── Encoder 3 — Motor FR (front-right) ──────────────────────── */
#define ENCODER3_PIN_A   22
#define ENCODER3_PIN_B   26

/* ── Encoder 4 — Motor RR (rear-right) ───────────────────────── */
#define ENCODER4_PIN_A   27
#define ENCODER4_PIN_B   28

/***************************************************************
 * Encoder Parameters
 ***************************************************************/

// Encoder counts per revolution (CPR)
#define ENCODER_CPR 374

// Radius of the wheel in centimeters
#define WHEEL_RADIUS_CM 4.25f

#endif // ENCODER_CONFIG_HPP
