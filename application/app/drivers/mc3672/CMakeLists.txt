# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_MC3672              mc3672.c)
zephyr_library_sources_ifdef(CONFIG_MC3672_TRIGGER      mc3672_trigger.c)