#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "gnss-config.h"
#include "log.h"

#define CHECK(condition)                                                                    \
    do {                                                                                    \
        if (!(condition)) {                                                                 \
            fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            exit(EXIT_FAILURE);                                                             \
        }                                                                                   \
    } while (0)

struct expected_value {
    const char* name;
    uint64_t    value;
};

struct expected_config {
    const char*                  name;
    int                          major;
    int                          minor;
    int                          bands;
    int                          nkv;
    uint64_t                     hash;
    const struct expected_value* values;
};

static const struct expected_value values_l1l2[] = {
    {"CFG-SIGNAL-GPS_L2C_ENA", 1},
    {"CFG-SIGNAL-GAL_E5B_ENA", 1},
    {"CFG-SIGNAL-BDS_B2_ENA", 1},
    {"CFG-SIGNAL-GLO_L2_ENA", 1},
    {NULL, 0},
};

static const struct expected_value values_l1l5[] = {
    {"CFG-SIGNAL-GPS_L5_ENA", 1},
    {"CFG-SIGNAL-GAL_E5A_ENA", 1},
    {"CFG-SIGNAL-QZSS_L5_ENA", 1},
    {"CFG-SIGNAL-BDS_B2_ENA", 0},
    {"CFG-SIGNAL-GLO_L2_ENA", 0},
    {NULL, 0},
};

/* Resolved configs of the deployed F9T fleet. Any change here makes every such
 * host rewrite its config and hard-reset the receiver on upgrade. */
static const struct expected_config fleet[] = {
    {"F9T-00B FW 2.01", 2, 1, GNSS_BAND_L1 | GNSS_BAND_L2, 571, 0xe8fbabc789fd5e9bULL, values_l1l2},
    {"F9T-00B FW 2.20", 2, 20, GNSS_BAND_L1 | GNSS_BAND_L2, 604, 0xde0b7356db971c68ULL, values_l1l2},
    {"F9T-10B FW 2.20", 2, 20, GNSS_BAND_L1 | GNSS_BAND_L5, 604, 0xd20c480c81879bfaULL, values_l1l5},
    {"F9T-00B FW unknown", -1, -1, GNSS_BAND_L1 | GNSS_BAND_L2, 571, 0xe8fbabc789fd5e9bULL, values_l1l2},
    {"F9T-10B FW unknown", -1, -1, GNSS_BAND_L1 | GNSS_BAND_L5, 604, 0xd20c480c81879bfaULL, values_l1l5},
};

static uint64_t value_of(const UBLOXCFG_KEYVAL_t* kv)
{
    switch (UBLOXCFG_ID2SIZE(kv->id)) {
    case UBLOXCFG_SIZE_BIT:
        return kv->val._raw & 0x1;
    case UBLOXCFG_SIZE_ONE:
        return kv->val._raw & 0xff;
    case UBLOXCFG_SIZE_TWO:
        return kv->val._raw & 0xffff;
    case UBLOXCFG_SIZE_FOUR:
        return kv->val._raw & 0xffffffff;
    case UBLOXCFG_SIZE_EIGHT:
        return kv->val._raw;
    }
    CHECK(false);
    return 0;
}

static int compare_by_id(const void* a, const void* b)
{
    uint32_t ida = ((const UBLOXCFG_KEYVAL_t*)a)->id;
    uint32_t idb = ((const UBLOXCFG_KEYVAL_t*)b)->id;

    return (ida > idb) - (ida < idb);
}

static uint64_t fnv1a(uint64_t hash, uint64_t data, int nbytes)
{
    for (int i = 0; i < nbytes; i++) {
        hash ^= (data >> (8 * i)) & 0xff;
        hash *= 0x100000001b3ULL;
    }
    return hash;
}

static uint64_t config_hash(UBLOXCFG_KEYVAL_t* kv, int nkv)
{
    uint64_t hash = 0xcbf29ce484222325ULL;

    qsort(kv, nkv, sizeof(*kv), compare_by_id);
    for (int i = 0; i < nkv; i++) {
        CHECK(i == 0 || kv[i].id != kv[i - 1].id);
        CHECK(kv[i].val._raw == value_of(&kv[i]));
        hash = fnv1a(hash, kv[i].id, 4);
        hash = fnv1a(hash, value_of(&kv[i]), 8);
    }
    return hash;
}

static const UBLOXCFG_KEYVAL_t* find_key(const UBLOXCFG_KEYVAL_t* kv, int nkv, const char* name)
{
    const UBLOXCFG_ITEM_t* item = ubloxcfg_getItemByName(name);

    CHECK(item != NULL);
    for (int i = 0; i < nkv; i++)
        if (kv[i].id == item->id)
            return &kv[i];
    return NULL;
}

static void test_fleet_configs_unchanged(void)
{
    for (size_t i = 0; i < sizeof(fleet) / sizeof(fleet[0]); i++) {
        const struct expected_config* e   = &fleet[i];
        int                           nkv = 0;
        UBLOXCFG_KEYVAL_t*            kv  = get_default_value_from_config(&nkv, e->major, e->minor, e->bands);

        CHECK(kv != NULL);
        for (const struct expected_value* v = e->values; v->name != NULL; v++) {
            const UBLOXCFG_KEYVAL_t* found = find_key(kv, nkv, v->name);

            CHECK(found != NULL);
            CHECK(value_of(found) == v->value);
        }
        uint64_t hash = config_hash(kv, nkv);
        printf("%s: nkv=%d hash=0x%016llx\n", e->name, nkv, (unsigned long long)hash);
        CHECK(nkv == e->nkv);
        CHECK(hash == e->hash);
        free(kv);
    }
}

int main(void)
{
    log_set_level(LOG_ERROR);
    test_fleet_configs_unchanged();
    return 0;
}
