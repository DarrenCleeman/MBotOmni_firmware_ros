#ifndef CONFIG_MBOT_OMNI_CONFIG_H
#define CONFIG_MBOT_OMNI_CONFIG_H

// Hardware Parameters
#define GEAR_RATIO              78.0
#define ENCODER_RES             20.0  // Omni encoder resolution.

// MBot Omni Parameters
#define OMNI_WHEEL_RADIUS            0.096f    // Wheel radius in meters (96mm radius)
#define OMNI_BASE_RADIUS             0.14232f  // Distance from center to wheel in meters (5.603 in)
#define MOT_R                        0   // Right motor slot
#define MOT_B                        1   // Back motor slot
#define MOT_L                        2   // Left motor slot
#define NUM_MOT_SLOTS                3   // Total number of wheel motor slots

// Math constants for omni kinematics
#define SQRT3                        1.7320508075688772f
#define INV_SQRT3                    0.5773502691896258f

#endif  /* CONFIG_MBOT_OMNI_CONFIG_H */
