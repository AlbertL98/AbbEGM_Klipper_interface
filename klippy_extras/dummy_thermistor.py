# =============================================================================
# DUMMY THERMISTOR - Klipper Extra Module  
# =============================================================================

class DummySensor:
    def __init__(self, config, params=None):
        self.temperature = 200.0
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.callback = None
        self.timer = None

    def setup_minmax(self, min_temp, max_temp):
        pass

    def setup_callback(self, cb):
        self.callback = cb
        self.timer = self.reactor.register_timer(
            self._timer_callback, self.reactor.NOW)

    def _timer_callback(self, eventtime):
        if self.callback:
            self.callback(eventtime + 0.1, self.temperature)
        return eventtime + 1.0

    def get_report_time_delta(self):
        return 1.0


class DummyThermistorLoader:
    def __init__(self, config):
        self.printer = config.get_printer()
        pheaters = self.printer.load_object(config, 'heaters')
        pheaters.add_sensor_factory("dummy_thermistor",
                                    self._create_sensor)

    def _create_sensor(self, config, params=None):
        return DummySensor(config, params)


def load_config(config):
    return DummyThermistorLoader(config)
