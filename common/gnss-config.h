#ifndef OSCILLATORD_GNSS_CONFIG_H
#define OSCILLATORD_GNSS_CONFIG_H

#include <ff/ff_rx.h>
#include <ubloxcfg/ubloxcfg.h>

/* Signal band flags for receiver configuration selection */
#define GNSS_BAND_L1  (1 << 0)
#define GNSS_BAND_L2  (1 << 1)
#define GNSS_BAND_L5  (1 << 2)
#define GNSS_BAND_L6  (1 << 3)

/* Signals tracked per band in one UBX-RXM-RAWX epoch; other = not in the band table */
struct gnss_band_counts {
	int l1;
	int l2;
	int l5;
	int l6;
	int other;
};

bool check_gnss_config_in_ram(RX_t *rx, UBLOXCFG_KEYVAL_t *allKvCfg, int nAllKvCfg);
int gnss_receiver_bands(const UBLOXCFG_KEYVAL_t *defaults, int nDefaults);
UBLOXCFG_KEYVAL_t *get_default_value_from_config(int *nKv, int major, int minor, int bands);
int gnss_signal_band(uint8_t gnssId, uint8_t sigId);
bool gnss_count_rawx_bands(const uint8_t *msg, int size, struct gnss_band_counts *counts);

#endif /* OSCILLATORD_GNSS_CONFIG_H */
