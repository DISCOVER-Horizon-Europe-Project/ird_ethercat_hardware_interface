// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

// Loading the configuration from a YAML file. See config/ird_csv_example.yaml
// for the format, with every key documented.
//
// Each drive starts from the built-in IRD CSV defaults (make_ird_csv_drive),
// then the optional `drive_defaults` section is applied, then the drive's own
// keys; a key given at a later level replaces the earlier value (lists are
// replaced as a whole). Unknown keys are errors, so a misspelt key never goes
// unnoticed.

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__CONFIG_LOADER_HPP_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__CONFIG_LOADER_HPP_

#include <string>
#include <vector>

#include "ird_ethercat_hardware_interface/config.hpp"

namespace ird_ethercat_hardware_interface
{

/// Reads a configuration file into `config`. Returns one message per problem
/// found while reading (empty on success). The result still has to be
/// checked with validate().
std::vector<std::string> load_config_file(const std::string & path, SystemConfig & config);

/// Same, from YAML text.
std::vector<std::string> load_config_string(const std::string & yaml, SystemConfig & config);

}  // namespace ird_ethercat_hardware_interface

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__CONFIG_LOADER_HPP_
