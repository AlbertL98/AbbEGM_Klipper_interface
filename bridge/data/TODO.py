# E-Wert + aktueller Lookahead für TX-Log
e_val = 0.0
e_age = -1.0
if self.moonraker:
    ext = self.moonraker.get_extruder_state()
    e_val = ext.e_value
    e_age = ext.age_ms
    if (e_age > self.cfg.moonraker.stale_threshold_ms
            and e_age > 0
            and self._loop_count % 250 == 0):
        logger.warning(
            "BRIDGE: Extruder-E-Wert veraltet "
            "(age=%.0fms > threshold=%.0fms)",
            e_age,
            self.cfg.moonraker.stale_threshold_ms)




        # E-Wert für RX-Log
        e_val = 0.0
        e_age = -1.0
        if self.moonraker:
            ext = self.moonraker.get_extruder_state()
            e_val = ext.e_value
            e_age = ext.age_ms