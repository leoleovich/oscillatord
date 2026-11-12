#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ff/ff_ubx.h>

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
    const struct expected_value* signals;
};

/* Every CFG-SIGNAL key of the profile; the F9T lists match the time01 and time04 receiver dumps */
static const struct expected_value signals_f9t_00b_v201[] = {
    {"CFG-SIGNAL-GPS_ENA", 1},       {"CFG-SIGNAL-GPS_L1CA_ENA", 1},  {"CFG-SIGNAL-GPS_L2C_ENA", 1},
    {"CFG-SIGNAL-SBAS_ENA", 1},      {"CFG-SIGNAL-SBAS_L1CA_ENA", 0}, {"CFG-SIGNAL-GAL_ENA", 1},
    {"CFG-SIGNAL-GAL_E1_ENA", 1},    {"CFG-SIGNAL-GAL_E5B_ENA", 1},   {"CFG-SIGNAL-BDS_ENA", 1},
    {"CFG-SIGNAL-BDS_B1_ENA", 1},    {"CFG-SIGNAL-BDS_B2_ENA", 1},    {"CFG-SIGNAL-QZSS_ENA", 1},
    {"CFG-SIGNAL-QZSS_L1CA_ENA", 1}, {"CFG-SIGNAL-QZSS_L1S_ENA", 0},  {"CFG-SIGNAL-QZSS_L2C_ENA", 1},
    {"CFG-SIGNAL-GLO_ENA", 1},       {"CFG-SIGNAL-GLO_L1_ENA", 1},    {"CFG-SIGNAL-GLO_L2_ENA", 1},
    {NULL, 0},
};

static const struct expected_value signals_f9t_00b_v220[] = {
    {"CFG-SIGNAL-GPS_ENA", 1},       {"CFG-SIGNAL-GPS_L1CA_ENA", 1},  {"CFG-SIGNAL-GPS_L2C_ENA", 1},
    {"CFG-SIGNAL-SBAS_ENA", 1},      {"CFG-SIGNAL-SBAS_L1CA_ENA", 0}, {"CFG-SIGNAL-GAL_ENA", 1},
    {"CFG-SIGNAL-GAL_E1_ENA", 1},    {"CFG-SIGNAL-GAL_E5B_ENA", 1},   {"CFG-SIGNAL-BDS_ENA", 1},
    {"CFG-SIGNAL-BDS_B1_ENA", 1},    {"CFG-SIGNAL-BDS_B2_ENA", 1},    {"CFG-SIGNAL-QZSS_ENA", 1},
    {"CFG-SIGNAL-QZSS_L1CA_ENA", 1}, {"CFG-SIGNAL-QZSS_L2C_ENA", 1},  {"CFG-SIGNAL-GLO_ENA", 1},
    {"CFG-SIGNAL-GLO_L1_ENA", 1},    {"CFG-SIGNAL-GLO_L2_ENA", 1},    {NULL, 0},
};

static const struct expected_value signals_f9t_10b[] = {
    {"CFG-SIGNAL-GPS_ENA", 1},       {"CFG-SIGNAL-GPS_L1CA_ENA", 1},  {"CFG-SIGNAL-GPS_L2C_ENA", 0},
    {"CFG-SIGNAL-GPS_L5_ENA", 1},    {"CFG-SIGNAL-SBAS_ENA", 1},      {"CFG-SIGNAL-SBAS_L1CA_ENA", 0},
    {"CFG-SIGNAL-GAL_ENA", 1},       {"CFG-SIGNAL-GAL_E1_ENA", 1},    {"CFG-SIGNAL-GAL_E5A_ENA", 1},
    {"CFG-SIGNAL-GAL_E5B_ENA", 0},   {"CFG-SIGNAL-BDS_ENA", 1},       {"CFG-SIGNAL-BDS_B1_ENA", 1},
    {"CFG-SIGNAL-BDS_B1C_ENA", 0},   {"CFG-SIGNAL-BDS_B2_ENA", 0},    {"CFG-SIGNAL-BDS_B2A_ENA", 1},
    {"CFG-SIGNAL-QZSS_ENA", 1},      {"CFG-SIGNAL-QZSS_L1CA_ENA", 1}, {"CFG-SIGNAL-QZSS_L2C_ENA", 0},
    {"CFG-SIGNAL-QZSS_L5_ENA", 1},   {"CFG-SIGNAL-GLO_ENA", 1},       {"CFG-SIGNAL-GLO_L1_ENA", 1},
    {"CFG-SIGNAL-GLO_L2_ENA", 0},    {"CFG-SIGNAL-NAVIC_ENA", 1},     {"CFG-SIGNAL-NAVIC_L5_ENA", 1},
    {NULL, 0},
};

/* Resolved configs of the deployed F9T fleet. Any change here makes every such
 * host rewrite its config and hard-reset the receiver on upgrade. */
static const struct expected_config fleet[] = {
    {"F9T-00B FW 2.01", 2, 1, GNSS_BAND_L1 | GNSS_BAND_L2, 571, 0xe8fbabc789fd5e9bULL, signals_f9t_00b_v201},
    {"F9T-00B FW 2.20", 2, 20, GNSS_BAND_L1 | GNSS_BAND_L2, 604, 0xde0b7356db971c68ULL, signals_f9t_00b_v220},
    {"F9T-10B FW 2.20", 2, 20, GNSS_BAND_L1 | GNSS_BAND_L5, 611, 0x483c658b9c192192ULL, signals_f9t_10b},
    {"F9T-00B FW unknown", -1, -1, GNSS_BAND_L1 | GNSS_BAND_L2, 571, 0xe8fbabc789fd5e9bULL, signals_f9t_00b_v201},
    {"F9T-10B FW unknown", -1, -1, GNSS_BAND_L1 | GNSS_BAND_L5, 611, 0x483c658b9c192192ULL, signals_f9t_10b},
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

static void check_signals(const UBLOXCFG_KEYVAL_t* kv, int nkv, const struct expected_value* signals)
{
    int nexpected = 0;
    int nsignal   = 0;

    for (const struct expected_value* v = signals; v->name != NULL; v++, nexpected++) {
        const UBLOXCFG_KEYVAL_t* found = find_key(kv, nkv, v->name);

        CHECK(found != NULL);
        CHECK(value_of(found) == v->value);
    }
    for (int i = 0; i < nkv; i++)
        if (UBLOXCFG_ID2GROUP(kv[i].id) == UBLOXCFG_CFG_SIGNAL_ID)
            nsignal++;
    CHECK(nsignal == nexpected);
}

static void test_fleet_configs_unchanged(void)
{
    for (size_t i = 0; i < sizeof(fleet) / sizeof(fleet[0]); i++) {
        const struct expected_config* e   = &fleet[i];
        int                           nkv = 0;
        UBLOXCFG_KEYVAL_t*            kv  = get_default_value_from_config(&nkv, e->major, e->minor, e->bands);

        CHECK(kv != NULL);
        check_signals(kv, nkv, e->signals);
        uint64_t hash = config_hash(kv, nkv);
        printf("%s: nkv=%d hash=0x%016llx\n", e->name, nkv, (unsigned long long)hash);
        CHECK(nkv == e->nkv);
        CHECK(hash == e->hash);
        free(kv);
    }
}

static void test_unsupported_bands_rejected(void)
{
    int nkv = 0;

    CHECK(get_default_value_from_config(&nkv, 2, 20, GNSS_BAND_L1) == NULL);
    CHECK(get_default_value_from_config(&nkv, 2, 20, GNSS_BAND_L1 | GNSS_BAND_L2 | GNSS_BAND_L5) == NULL);
    CHECK(get_default_value_from_config(&nkv, 2, 20, GNSS_BAND_L1 | GNSS_BAND_L6) == NULL);
}

static void test_x20t_config(void)
{
    static const struct expected_value signals_x20t[] = {
        {"CFG-SIGNAL-GPS_ENA", 1},       {"CFG-SIGNAL-GPS_L1CA_ENA", 1},  {"CFG-SIGNAL-GPS_L2C_ENA", 1},
        {"CFG-SIGNAL-GPS_L5_ENA", 1},    {"CFG-SIGNAL-SBAS_ENA", 1},      {"CFG-SIGNAL-SBAS_L1CA_ENA", 0},
        {"CFG-SIGNAL-GAL_ENA", 1},       {"CFG-SIGNAL-GAL_E1_ENA", 1},    {"CFG-SIGNAL-GAL_E5A_ENA", 1},
        {"CFG-SIGNAL-GAL_E6_ENA", 1},    {"CFG-SIGNAL-BDS_ENA", 1},       {"CFG-SIGNAL-BDS_B1_ENA", 1},
        {"CFG-SIGNAL-BDS_B1C_ENA", 0},   {"CFG-SIGNAL-BDS_B2A_ENA", 1},   {"CFG-SIGNAL-BDS_B3_ENA", 1},
        {"CFG-SIGNAL-QZSS_ENA", 1},      {"CFG-SIGNAL-QZSS_L1CA_ENA", 1}, {"CFG-SIGNAL-QZSS_L2C_ENA", 1},
        {"CFG-SIGNAL-QZSS_L5_ENA", 1},   {"CFG-SIGNAL-GLO_ENA", 1},       {"CFG-SIGNAL-GLO_L1_ENA", 1},
        {"CFG-SIGNAL-GLO_L2_ENA", 1},    {"CFG-SIGNAL-NAVIC_ENA", 1},     {"CFG-SIGNAL-NAVIC_L5_ENA", 1},
        {NULL, 0},
    };
    const int bands = GNSS_BAND_L1 | GNSS_BAND_L2 | GNSS_BAND_L5 | GNSS_BAND_L6;
    int       nkv   = 0;
    UBLOXCFG_KEYVAL_t* kv = get_default_value_from_config(&nkv, -1, -1, bands);

    CHECK(kv != NULL);
    check_signals(kv, nkv, signals_x20t);
    uint64_t hash = config_hash(kv, nkv);
    printf("ZED-X20T: nkv=%d hash=0x%016llx\n", nkv, (unsigned long long)hash);
    CHECK(nkv == 611);
    CHECK(hash == 0xc24d091eebf00758ULL);
    free(kv);
}

static void test_receiver_bands(void)
{
    static const UBLOXCFG_KEYVAL_t l2c_only[] = {
        {.id = UBLOXCFG_CFG_SIGNAL_GPS_L2C_ENA_ID, .val.L = true},
    };
    static const UBLOXCFG_KEYVAL_t l2c_on_with_l5[] = {
        {.id = UBLOXCFG_CFG_SIGNAL_GPS_L2C_ENA_ID, .val.L = true},
        {.id = UBLOXCFG_CFG_SIGNAL_GPS_L5_ENA_ID, .val.L = false},
    };
    static const UBLOXCFG_KEYVAL_t l2c_off_without_l5[] = {
        {.id = UBLOXCFG_CFG_SIGNAL_GPS_L2C_ENA_ID, .val.L = false},
    };
    /* Factory defaults read from the F9T-10B on time04 */
    static const UBLOXCFG_KEYVAL_t f9t_10b[] = {
        {.id = UBLOXCFG_CFG_SIGNAL_GPS_L5_ENA_ID, .val.L = false},
        {.id = UBLOXCFG_CFG_SIGNAL_GPS_L2C_ENA_ID, .val.L = false},
    };
    static const UBLOXCFG_KEYVAL_t with_e6[] = {
        {.id = UBLOXCFG_CFG_SIGNAL_GPS_L2C_ENA_ID, .val.L = true},
        {.id = UBLOXCFG_CFG_SIGNAL_GPS_L5_ENA_ID, .val.L = false},
        {.id = UBLOXCFG_CFG_SIGNAL_GAL_E6_ENA_ID, .val.L = true},
    };
    static const UBLOXCFG_KEYVAL_t e6_off[] = {
        {.id = UBLOXCFG_CFG_SIGNAL_GAL_E6_ENA_ID, .val.L = false},
    };
    const int all = GNSS_BAND_L1 | GNSS_BAND_L2 | GNSS_BAND_L5 | GNSS_BAND_L6;

    CHECK(gnss_receiver_bands(NULL, -1) == (GNSS_BAND_L1 | GNSS_BAND_L2));
    CHECK(gnss_receiver_bands(NULL, 0) == (GNSS_BAND_L1 | GNSS_BAND_L2));
    CHECK(gnss_receiver_bands(l2c_only, 1) == (GNSS_BAND_L1 | GNSS_BAND_L2));
    CHECK(gnss_receiver_bands(l2c_on_with_l5, 2) == (GNSS_BAND_L1 | GNSS_BAND_L2));
    CHECK(gnss_receiver_bands(l2c_off_without_l5, 1) == (GNSS_BAND_L1 | GNSS_BAND_L2));
    CHECK(gnss_receiver_bands(f9t_10b, 2) == (GNSS_BAND_L1 | GNSS_BAND_L5));
    CHECK(gnss_receiver_bands(with_e6, 3) == all);
    CHECK(gnss_receiver_bands(e6_off, 1) == all);
}

struct signal_band {
    uint8_t gnssId;
    uint8_t sigId;
    int     band;
};

/* gnssId/sigId pairs of the u-blox interface descriptions, as numbers */
static const struct signal_band known_signals[] = {
    {0, 0, GNSS_BAND_L1},  {0, 3, GNSS_BAND_L2},  {0, 4, GNSS_BAND_L2},  {0, 6, GNSS_BAND_L5},
    {0, 7, GNSS_BAND_L5},  {1, 0, GNSS_BAND_L1},  {2, 0, GNSS_BAND_L1},  {2, 1, GNSS_BAND_L1},
    {2, 3, GNSS_BAND_L5},  {2, 4, GNSS_BAND_L5},  {2, 5, GNSS_BAND_L2},  {2, 6, GNSS_BAND_L2},
    {2, 8, GNSS_BAND_L6},  {2, 9, GNSS_BAND_L6},  {2, 10, GNSS_BAND_L6}, {3, 0, GNSS_BAND_L1},
    {3, 1, GNSS_BAND_L1},  {3, 2, GNSS_BAND_L2},  {3, 3, GNSS_BAND_L2},  {3, 4, GNSS_BAND_L6},
    {3, 5, GNSS_BAND_L1},  {3, 6, GNSS_BAND_L1},  {3, 7, GNSS_BAND_L5},  {3, 8, GNSS_BAND_L5},
    {3, 10, GNSS_BAND_L6}, {5, 0, GNSS_BAND_L1},  {5, 1, GNSS_BAND_L1},  {5, 4, GNSS_BAND_L2},
    {5, 5, GNSS_BAND_L2},  {5, 8, GNSS_BAND_L5},  {5, 9, GNSS_BAND_L5},  {6, 0, GNSS_BAND_L1},
    {6, 2, GNSS_BAND_L2},  {7, 0, GNSS_BAND_L5},
};

static const struct signal_band unknown_signals[] = {
    {0, 1, 0},  {0, 2, 0},  {0, 5, 0},  {1, 1, 0},  {1, 3, 0}, {2, 2, 0},
    {2, 7, 0},  {2, 11, 0}, {3, 9, 0},  {3, 11, 0}, {5, 2, 0}, {5, 12, 0},
    {6, 1, 0},  {7, 1, 0},  {4, 0, 0},  {0xff, 0, 0},
};

static void test_signal_bands(void)
{
    for (size_t i = 0; i < sizeof(known_signals) / sizeof(known_signals[0]); i++)
        CHECK(gnss_signal_band(known_signals[i].gnssId, known_signals[i].sigId) == known_signals[i].band);
    for (size_t i = 0; i < sizeof(unknown_signals) / sizeof(unknown_signals[0]); i++)
        CHECK(gnss_signal_band(unknown_signals[i].gnssId, unknown_signals[i].sigId) == 0);
}

struct rawx_meas {
    uint8_t gnssId;
    uint8_t sigId;
    uint8_t cno;
};

static int make_rawx(uint8_t* frame, uint8_t version, const struct rawx_meas* meas, int n)
{
    UBX_RXM_RAWX_V1_GROUP0_t head = {.numMeas = n, .version = version};
    uint8_t                  payload[sizeof(head) + 255 * sizeof(UBX_RXM_RAWX_V1_GROUP1_t)];

    memcpy(payload, &head, sizeof(head));
    for (int i = 0; i < n; i++) {
        UBX_RXM_RAWX_V1_GROUP1_t m = {.gnssId = meas[i].gnssId, .sigId = meas[i].sigId, .cno = meas[i].cno};

        memcpy(&payload[sizeof(head) + i * sizeof(m)], &m, sizeof(m));
    }
    return ubxMakeMessage(UBX_RXM_CLSID, UBX_RXM_RAWX_MSGID, payload,
                          sizeof(head) + n * sizeof(UBX_RXM_RAWX_V1_GROUP1_t), frame);
}

static bool counts_equal(const struct gnss_band_counts* c, int l1, int l2, int l5, int l6, int other)
{
    return c->l1 == l1 && c->l2 == l2 && c->l5 == l5 && c->l6 == l6 && c->other == other;
}

static void test_count_rawx_bands(void)
{
    static const struct rawx_meas mixed[] = {
        {0, 0, 40},  /* GPS L1C/A */
        {0, 3, 35},  /* GPS L2 CL */
        {2, 4, 30},  /* Galileo E5a Q */
        {2, 8, 28},  /* Galileo E6 B */
        {3, 4, 25},  /* BeiDou B3I D1 */
        {7, 0, 33},  /* NavIC L5 A */
        {5, 12, 20}, /* unknown QZSS signal */
        {0, 0, 0},   /* not tracked */
    };
    const struct gnss_band_counts untouched = {7, 7, 7, 7, 7};
    struct gnss_band_counts       counts;
    uint8_t frame[UBX_FRAME_SIZE + sizeof(UBX_RXM_RAWX_V1_GROUP0_t) + 255 * sizeof(UBX_RXM_RAWX_V1_GROUP1_t)];
    int     size = make_rawx(frame, UBX_RXM_RAWX_V1_VERSION, mixed, 8);

    CHECK(gnss_count_rawx_bands(frame, size, &counts));
    CHECK(counts_equal(&counts, 1, 1, 2, 2, 1));

    counts = untouched;
    CHECK(!gnss_count_rawx_bands(frame, size - 1, &counts));
    CHECK(!gnss_count_rawx_bands(frame, size + 1, &counts));
    CHECK(!gnss_count_rawx_bands(frame, UBX_FRAME_SIZE, &counts));
    CHECK(memcmp(&counts, &untouched, sizeof(counts)) == 0);

    size = make_rawx(frame, 0, mixed, 8);
    CHECK(!gnss_count_rawx_bands(frame, size, &counts));
    CHECK(memcmp(&counts, &untouched, sizeof(counts)) == 0);

    size = make_rawx(frame, UBX_RXM_RAWX_V1_VERSION, mixed, 0);
    CHECK(gnss_count_rawx_bands(frame, size, &counts));
    CHECK(counts_equal(&counts, 0, 0, 0, 0, 0));
}

int main(void)
{
    log_set_level(LOG_FATAL);
    test_fleet_configs_unchanged();
    test_unsupported_bands_rejected();
    test_x20t_config();
    test_receiver_bands();
    test_signal_bands();
    test_count_rawx_bands();
    return 0;
}
