/* Signal database for the simulator: every message field by name, bit position, type and scale
 * (the tables in sbt CAN.md 4.1). Used by the console to encode/decode frames by field name, and by
 * the tests as an independent second decoder to cross-check the firmware codec (can_msgs.c). */
#ifndef SIM_DBC_H
#define SIM_DBC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char *name;
    uint8_t start;          /* bit 0 = bit 0 of byte 0, little-endian */
    uint8_t bits;
    bool is_signed;
    double scale;           /* physical = raw * scale */
    uint8_t decimals;       /* for printing */
    const char *unit;
    const char *const *enum_names;   /* optional: names for raw values 0..n_enum-1 */
    uint8_t n_enum;
} sim_field_t;

typedef struct {
    uint16_t id;
    const char *name;
    uint8_t dlc;
    const char *sender;
    const sim_field_t *fields;
    uint8_t n_fields;
} sim_msg_t;

uint8_t sim_dbc_count(void);
const sim_msg_t *sim_dbc_at(uint8_t index);
const sim_msg_t *sim_dbc_by_id(uint16_t id);
const sim_msg_t *sim_dbc_by_name(const char *name);      /* case-insensitive */
int sim_dbc_field_index(const sim_msg_t *msg, const char *field);   /* -1 if unknown */

int64_t sim_dbc_get_raw(const sim_field_t *f, const uint8_t *data);
double sim_dbc_get(const sim_field_t *f, const uint8_t *data);
/* Encodes value; returns false if it had to be clamped to the field's bit range. */
bool sim_dbc_set(const sim_field_t *f, uint8_t *data, double value);

/* Parses a number or (case-insensitive) enum name. */
bool sim_dbc_parse_value(const sim_field_t *f, const char *text, double *out);
/* Formats one field value, e.g. "25.3" or "SAFE_STATE". */
void sim_dbc_format_value(const sim_field_t *f, double value, char *buf, size_t n);
/* Formats all fields, e.g. "motor_temp=25.3 C ctrl_temp=24.0 C". */
void sim_dbc_format(const sim_msg_t *msg, const uint8_t *data, char *buf, size_t n);

#endif
