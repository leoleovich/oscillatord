/**
 * @file art_disciplining_manager.c
 * @brief Read and Write dsiciplining config to ART card's EEPROM use disciplining_config_file
 * @version 0.1
 * @date 2022-01-24
 *
 */
#include <getopt.h>
#include <errno.h>
#include <error.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <oscillator-disciplining/oscillator-disciplining.h>

#include "config.h"
#include "eeprom_config.h"
#include "eeprom.h"
#include "log.h"

enum Mode {
    ART_EEPROM_MANAGER_NONE,
    ART_EEPROM_MANAGER_READ,
    ART_EEPROM_MANAGER_WRITE,
    ART_EEPROM_MANAGER_INIT
};

struct disciplining_config factory_config = {
    .header = HEADER_MAGIC,
    .version = 1,
    .ctrl_nodes_length = 3,
    .ctrl_load_nodes = {0.25,0.5,0.75},
    .ctrl_drift_coeffs = {0.0,0.0,0.0},
    .coarse_equilibrium = -1,
    .ctrl_nodes_length_factory = 3,
    .ctrl_load_nodes_factory = {0.25,0.5,0.75},
    .ctrl_drift_coeffs_factory = {1.2,0.0,-1.2},
    .coarse_equilibrium_factory = -1,
    .calibration_valid = false,
    .calibration_date = 0
};

static int write_disciplining_parameters_to_disciplining_config_file(const char * path, struct disciplining_config *config)
{
    char buffer[DISCIPLINING_CONFIG_FILE_SIZE];
    int ret;

    if (config == NULL) {
        log_error("config is NULL");
        return -EINVAL;
    }

    memset(buffer, 0, DISCIPLINING_CONFIG_FILE_SIZE * sizeof(char));
    memcpy(buffer, config, sizeof(struct disciplining_config_V_1));

    ret = write_file((char *) path, buffer, DISCIPLINING_CONFIG_FILE_SIZE);
    if (ret != 0) {
        log_error("Could not write data in %s", path);
        return -1;
    }

    return 0;
}

static int read_disciplining_parameters_from_disciplining_config_file(const char *path, struct disciplining_config *config)
{
    char buffer[DISCIPLINING_CONFIG_FILE_SIZE];
    int dsc_config_version = 0;
    int ret;

    ret = read_file((char *) path, (char *) buffer, DISCIPLINING_CONFIG_FILE_SIZE);
    if (ret != 0) {
        log_error("Could not read disciplining config at %s", path);
        return -1;
    }
    if (check_header_valid(buffer[0])) {
        dsc_config_version = buffer[1];
        log_info("Version of disciplining_config file: %d", dsc_config_version);
        if (dsc_config_version == 1) {
            /*
             * Data in files is stored in format version 1
             * fill struct disciplining_parameters
             */
            memcpy(config, buffer, sizeof(struct disciplining_config_V_1));
            return 0;
        } else {
            log_error("Unknown version %d", dsc_config_version);
            return -1;
        }
    } else {
        log_error("Header in %s is not valid !", path);
        log_error("Please upgrade disciplining_config and temperature table using art_eeprom_data_updater !");
        return -1;
    }
}

static int double_array_parser(const char* value, double **result) {
    char *endptr;
    char *ptr;
    const char *delim = ",";
    double buffer[CALIBRATION_POINTS_MAX];
    int parsed = 0;
    double value_double;

    errno = 0;

    ptr = strtok((char *) value, delim);
    while (ptr != NULL)
    {
        if (parsed >= CALIBRATION_POINTS_MAX) {
            return -ERANGE;
        }
        value_double = strtold(ptr, &endptr);
        if (value_double == HUGE_VAL ||
            (value_double == 0 && errno == ERANGE))
            return -ERANGE;
        buffer[parsed] = value_double;
        parsed++;
        ptr = strtok(NULL, delim);
    }

    double *values = malloc(parsed * sizeof(double));
    if (values == NULL) {
        return -ENOMEM;
    }
    for (int i = 0; i < parsed; i++) {
        values[i] = buffer[i];
    }

    *result = values;

    return parsed;
}

static double * get_double_array_from_config(struct config *config, const char *key, int expected_length)
{
    int ret;
    const char *value;
    char *value_cpy = NULL;
    double *result = NULL;

    value = config_get(config, key);
    if (value == NULL) {
        log_error("Error reading %s", key);
        return NULL;
    }
    value_cpy = strdup(value);
    ret = double_array_parser(value_cpy, &result);
    free(value_cpy);
    if (ret != expected_length) {
        log_error("Error: Expected length of %d for %s. Got %d", expected_length, key, ret);
        free(result);
        return NULL;
    }

    return result;
}

static int read_disciplining_parameters_from_file(const char *path, struct disciplining_config *dsc_config)
{
    double *ctrl_drift_coeffs;
    double *ctrl_load_nodes;
    struct config config;
    int ret;
    int32_t factory_coarse = 0;

    ret = config_init(&config, path);
    if (ret != 0)
        error(EXIT_FAILURE, -ret, "config_init(%s)", path);

    memcpy(dsc_config, &factory_config, sizeof(struct disciplining_config));

    dsc_config->calibration_valid = config_get_bool_default(&config, "calibration_valid", false);
    dsc_config->coarse_equilibrium = atoi(config_get_default(&config, "coarse_equilibrium", "-1"));
    factory_coarse = atoi(config_get_default(&config, "coarse_equilibrium_factory", "-1"));
    if (factory_coarse > 0) {
        log_info("Update coarse equilibrium factory to %d", factory_coarse);
        dsc_config->coarse_equilibrium_factory = factory_coarse;
    }


    if (config_get_unsigned_number(&config, "ctrl_nodes_length") > 0) {
        dsc_config->ctrl_nodes_length = config_get_unsigned_number(&config, "ctrl_nodes_length");
    } else {
        log_error("error parsing key ctrl_nodes_length, aborting");
        return -1;
    }

    if (config_get_unsigned_number(&config, "calibration_date") > 0)
        dsc_config->calibration_date = config_get_unsigned_number(&config, "calibration_date");
    else
        dsc_config->calibration_date = time(NULL);


    ctrl_load_nodes = get_double_array_from_config(&config, "ctrl_load_nodes", dsc_config->ctrl_nodes_length);
    if (ctrl_load_nodes == NULL) {
        log_error("Could not get ctrl_load_nodes from config file at %s", path);
        return -1;
    }
    ctrl_drift_coeffs = get_double_array_from_config(&config, "ctrl_drift_coeffs", dsc_config->ctrl_nodes_length);
    if (ctrl_drift_coeffs == NULL) {
        log_error("Could not get ctrl_drift_coeffs from config file at %s", path);
        free(ctrl_load_nodes);
        return -1;
    }
    for (uint i = 0; i < dsc_config->ctrl_nodes_length; i++) {
        dsc_config->ctrl_load_nodes[i] = ctrl_load_nodes[i];
        dsc_config->ctrl_drift_coeffs[i] = ctrl_drift_coeffs[i];
    }

    if (config_get_unsigned_number(&config, "estimated_equilibrium_ES") > 0) {
        dsc_config->estimated_equilibrium_ES = config_get_unsigned_number(&config, "estimated_equilibrium_ES");
    } else {
        log_warn("Could not find key estimated_equilibrium_ES, setting value to 0");
        dsc_config->estimated_equilibrium_ES = 0;
    }

    log_info("Disciplining parameters that written from %s:", path);
    print_disciplining_config(dsc_config, LOG_INFO);

    free(ctrl_drift_coeffs);
    free(ctrl_load_nodes);
    config_cleanup(&config);

    return 0;
}

static int write_disciplining_parameters_to_file(const char *path, struct disciplining_config *dsc_config)
{
    struct config config;
    char buffer[2048];
    char float_buffer[256];

    memset(&config, 0, sizeof(config));
    memset(buffer, 0, sizeof(buffer));
    sprintf(buffer, "%d", dsc_config->coarse_equilibrium);
    config_set(&config, "coarse_equilibrium", buffer);


    sprintf(buffer, "%u", dsc_config->ctrl_nodes_length);
    config_set(&config, "ctrl_nodes_length", buffer);

    memset(buffer, 0, sizeof(buffer));
    for (int i = 0; i < dsc_config->ctrl_nodes_length; i++) {
        sprintf(float_buffer, "%f", dsc_config->ctrl_load_nodes[i]);
        strncat(buffer, float_buffer, strlen(float_buffer));
        if (i != dsc_config->ctrl_nodes_length - 1)
            strcat(buffer, ",");
    }
    config_set(&config, "ctrl_load_nodes", buffer);

    memset(buffer, 0, sizeof(buffer));
    for (int i = 0; i < dsc_config->ctrl_nodes_length; i++) {
        sprintf(float_buffer, "%f", dsc_config->ctrl_drift_coeffs[i]);
        strncat(buffer, float_buffer, strlen(float_buffer));
        if (i != dsc_config->ctrl_nodes_length - 1)
            strcat(buffer, ",");
    }
    config_set(&config, "ctrl_drift_coeffs", buffer);

        sprintf(buffer, "%d", dsc_config->coarse_equilibrium_factory);
    config_set(&config, "coarse_equilibrium_factory", buffer);

    sprintf(buffer, "%s", dsc_config->calibration_valid ? "true" : "false");
    config_set(&config, "calibration_valid", buffer);

    sprintf(buffer, "%ld", dsc_config->calibration_date);
    config_set(&config, "calibration_date", buffer);

    sprintf(buffer, "%d\n", dsc_config->estimated_equilibrium_ES);
    config_set(&config, "estimated_equilibrium_ES", buffer);

    config_dump(&config, buffer, 2048);

    FILE *fd = fopen(path, "w+");
    int return_val = fputs(buffer,fd);
    fclose(fd);
    return return_val == 1 ? 0 : -1;
}

int main(int argc, char *argv[])
{
    enum Mode mode = ART_EEPROM_MANAGER_NONE;
    struct disciplining_config dsc_config;
    char *input_dsc_config_path = NULL;
    char *output_file = NULL;
    char *path = NULL;
    int option;
    int ret = 0;
    log_set_level(LOG_INFO);

    while ((option = getopt(argc, argv, ":m:p:w:fho:r")) != -1) {
        switch (option) {
        case 'r':
            mode = ART_EEPROM_MANAGER_READ;
            break;
        case 'w':
            mode = ART_EEPROM_MANAGER_WRITE;
            input_dsc_config_path = optarg;
            break;
        case 'o':
            output_file = optarg;
            break;
        case 'f':
            mode = ART_EEPROM_MANAGER_INIT;
            break;
        case 'p':
            path = optarg;
            break;
        case ':':
            log_error("Option needs a value ");
            break;
        case 'h':
        default:
            log_info("art_disciplining_manager -p disciplining_config_file_path [-w disciplining_config.txt | -r -o output_file_path | -f ] -h]");
            log_info("\t-p disciplining_config_file_path: Path to the disciplining_config file exposed by the driver");
            log_info("\t-w disciplining_config.txt: Path to the disciplining_config file to write in the eeprom");
            log_info("\t-r: Read disciplining_config from the eeprom");
            log_info("\t-f: Write factory parameters");
            log_info("\t-o: output_file_path: write disciplining_config read in file");
            log_info("\t-h: print help");
            return 0;
        }
    }

    if (path == NULL) {
        log_error("No eeprom/mro50 path provided!");
        return -1;
    }

    log_debug("Path: %s", path);
    log_debug("Mode: %d", mode);

    switch (mode) {
    case ART_EEPROM_MANAGER_READ:
        log_info("Reading data from %s:", path);
        ret = read_disciplining_parameters_from_disciplining_config_file(path, &dsc_config);
        if (ret != 0) {
            log_error("Could not read disciplining_config from %s", path);
            return -1;
        }
        print_disciplining_config(&dsc_config, LOG_INFO);
        if (output_file) {
            log_info("Writing disciplining parameters read to %s", output_file);
            ret = write_disciplining_parameters_to_file(output_file, &dsc_config);
            if (ret != 0) {
                log_error("Error writing in config file");
            }
        }
        break;
    case ART_EEPROM_MANAGER_WRITE:
        log_info("Writing calibration from %s to %s",input_dsc_config_path, path);
        ret = read_disciplining_parameters_from_file(input_dsc_config_path, &dsc_config);
        if (ret != 0) {
            log_info("Error reading input calibration from %s", input_dsc_config_path);
            return -1;
        }
        ret = write_disciplining_parameters_to_disciplining_config_file(path, &dsc_config);
        if (ret != 0) {
            log_error("Error writing calibration parameters to %s", path);
        }
        break;
    case ART_EEPROM_MANAGER_INIT:
        log_info("Writing default calibration to %s", path);
        write_disciplining_parameters_to_disciplining_config_file(path, (struct disciplining_config *) &factory_config);
        break;
    default:
        log_error("No Mode (Read, Write or Init) provided");
        return -1;
    }

    log_info("Success");
    return ret;
}
