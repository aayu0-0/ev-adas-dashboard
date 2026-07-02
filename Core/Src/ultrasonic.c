#include "ultrasonic.h"

typedef struct {
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t      echo_pin;
} HCSR04_Sensor_t;

static const HCSR04_Sensor_t SENSORS[HCSR04_COUNT] = {
    [HCSR04_FRONT] = { FRONT_TRIG_PORT, FRONT_TRIG_PIN,
                        FRONT_ECHO_PORT, FRONT_ECHO_PIN },
    [HCSR04_LEFT]  = { LEFT_TRIG_PORT,  LEFT_TRIG_PIN,
                        LEFT_ECHO_PORT,  LEFT_ECHO_PIN  },
    [HCSR04_RIGHT] = { RIGHT_TRIG_PORT, RIGHT_TRIG_PIN,
                        RIGHT_ECHO_PORT, RIGHT_ECHO_PIN },
};

static HCSR04_Result_t results[HCSR04_COUNT];

/* ── Plausibility filter state (per sensor) ──────────────────────────── */
static float   last_valid_cm[HCSR04_COUNT];
static uint8_t last_valid_init[HCSR04_COUNT];  /* 0 until first valid sample seen */
static float   pending_cm[HCSR04_COUNT];
static uint8_t pending_count[HCSR04_COUNT];

/* ── FIX 1: Cast to uint16_t for safe 16-bit rollover ─────────────────── */
static inline uint16_t TIM2_GetUS(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

static void Delay_US(uint16_t us)
{
    uint16_t start = TIM2_GetUS();
    while ((uint16_t)(TIM2_GetUS() - start) < us);
}

void HCSR04_Init(void)
{
    for (int i = 0; i < HCSR04_COUNT; i++) {
        results[i].distance_cm = HCSR04_MAX_CM;
        results[i].valid       = 0;
        HAL_GPIO_WritePin(SENSORS[i].trig_port,
                          SENSORS[i].trig_pin,
                          GPIO_PIN_RESET);

        last_valid_cm[i]   = HCSR04_MAX_CM;
        last_valid_init[i] = 0;
        pending_cm[i]      = 0.0f;
        pending_count[i]   = 0;
    }
}

/* ── Internal: plausibility filter ───────────────────────────────────── *
 * Called only on genuinely valid (non-timeout) raw readings. A timeout is
 * real information ("nothing in range") and is passed through untouched
 * in HCSR04_Read() — this filter only guards against noisy valid samples
 * jumping implausibly far from the last accepted value. */
static float plausibility_filter(HCSR04_ID id, float raw_cm)
{
    if (!last_valid_init[id]) {
        /* First sample ever for this sensor — nothing to compare against */
        last_valid_init[id] = 1;
        last_valid_cm[id]   = raw_cm;
        pending_count[id]   = 0;
        return raw_cm;
    }

    float diff = fabsf(raw_cm - last_valid_cm[id]);

    if (diff <= HCSR04_PLAUSIBILITY_JUMP_CM) {
        /* Believable change — accept immediately, reset any pending jump */
        last_valid_cm[id] = raw_cm;
        pending_count[id] = 0;
        return raw_cm;
    }

    /* Implausible jump — don't trust it yet. Check if it matches the
       previous pending sample (i.e. two big jumps in a row landing near
       each other = probably real, not noise). */
    if (pending_count[id] > 0 &&
        fabsf(raw_cm - pending_cm[id]) <= HCSR04_CONFIRM_TOLERANCE_CM) {
        pending_count[id]++;
    } else {
        pending_cm[id]    = raw_cm;
        pending_count[id] = 1;
    }

    if (pending_count[id] >= HCSR04_CONFIRM_SAMPLES) {
        /* Confirmed by consecutive agreeing samples — accept as real */
        last_valid_cm[id] = raw_cm;
        pending_count[id] = 0;
        return raw_cm;
    }

    /* Not yet confirmed — hold last known-good value instead of the spike */
    return last_valid_cm[id];
}

HCSR04_Result_t HCSR04_Read(HCSR04_ID id)
{
    HCSR04_Result_t res  = { HCSR04_MAX_CM, 0 };
    const HCSR04_Sensor_t *s = &SENSORS[id];

    /* ── Step 1: Send 10µs TRIG pulse ─────────────────────────────── */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
    Delay_US(2);                              /* ensure clean LOW start */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    Delay_US(10);                             /* 10µs HIGH              */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

    /* ── Step 2: Wait for ECHO HIGH ────────────────────────────────── */
    uint16_t t_start = TIM2_GetUS();
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET) {
        if ((uint16_t)(TIM2_GetUS() - t_start) > 30000U) {
            results[id] = res;               /* timeout — return invalid */
            /* Genuine "nothing in range" — reset the filter baseline so a
               real close obstacle right after this isn't mistaken for an
               implausible jump and needlessly held back. */
            last_valid_cm[id]   = HCSR04_MAX_CM;
            last_valid_init[id] = 1;
            pending_count[id]   = 0;
            return res;
        }
    }
    uint16_t echo_rise = TIM2_GetUS();

    /* ── Step 3: Wait for ECHO LOW ─────────────────────────────────── */
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET) {
        if ((uint16_t)(TIM2_GetUS() - echo_rise) > 30000U) {
            results[id] = res;
            last_valid_cm[id]   = HCSR04_MAX_CM;
            last_valid_init[id] = 1;
            pending_count[id]   = 0;
            return res;
        }
    }
    uint16_t echo_fall = TIM2_GetUS();

    /* ── Step 4: Calculate distance ────────────────────────────────── */
    uint16_t pulse_us  = echo_fall - echo_rise;
    float    dist_cm   = (pulse_us * SOUND_SPEED_CM_PER_US) / 2.0f;

    res.distance_cm = CLAMP(dist_cm, HCSR04_MIN_CM, HCSR04_MAX_CM);
    res.valid       = 1;

    /* Guard against single-cycle sensor noise (see plausibility_filter()
       above) before this value ever reaches adas.c/fault.c. */
    res.distance_cm = plausibility_filter(id, res.distance_cm);

    results[id]     = res;
    return res;
}

void HCSR04_ReadAll(void)
{
    for (int i = 0; i < HCSR04_COUNT; i++) {
        HCSR04_Read((HCSR04_ID)i);
        HAL_Delay(50);
    }
}

float   HCSR04_GetDistance(HCSR04_ID id) { return results[id].distance_cm; }
uint8_t HCSR04_IsValid    (HCSR04_ID id) { return results[id].valid;        }
