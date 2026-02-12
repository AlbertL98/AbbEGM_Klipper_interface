# =============================================================================
# EGM BRIDGE - Klipper Extra Module
# =============================================================================
# Datei: klippy_extras/egm_bridge.py
# 
# Wird beim Container-Start automatisch nach 
# /home/klippy/klipper/klippy/extras/ verlinkt.
# =============================================================================

import logging

class EgmBridge:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.logger = logging.getLogger('egm_bridge')
        
        # Config
        self.debug_mode = config.getboolean('debug_mode', True)
        self.log_moves = config.getboolean('log_moves', False)
        
        # Klipper Objekte
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = None
        
        # State
        self.move_count = 0
        self.is_active = False
        self.virtual_pos = [0.0, 0.0, 0.0, 0.0]  # X, Y, Z, E
        
        # Commands registrieren
        self.gcode.register_command('EGM_STATUS', self.cmd_EGM_STATUS,
            desc="Zeigt EGM Bridge Status")
        self.gcode.register_command('EGM_ACTIVATE', self.cmd_EGM_ACTIVATE,
            desc="Aktiviert EGM Bridge")
        self.gcode.register_command('EGM_DEACTIVATE', self.cmd_EGM_DEACTIVATE,
            desc="Deaktiviert EGM Bridge")
        self.gcode.register_command('EGM_TEST', self.cmd_EGM_TEST,
            desc="Test-Befehl mit Parametern")
        self.gcode.register_command('EGM_MOVE', self.cmd_EGM_MOVE,
            desc="Simulierter Move")
        self.gcode.register_command('EGM_MOVE_LOG', self.cmd_EGM_MOVE_LOG,
            desc="Toggle Move-Logging")
        
        # Events
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self._log("EGM Bridge initialisiert")
    
    def _handle_ready(self):
        try:
            self.toolhead = self.printer.lookup_object('toolhead')
            self._log("Ready - Toolhead verfügbar")
        except:
            self._log("Ready - Simulator-Modus (kein Toolhead)")
    
    def _log(self, msg):
        if self.debug_mode:
            self.logger.info(f"[EGM] {msg}")
    
    # === Commands ===
    
    def cmd_EGM_STATUS(self, gcmd):
        gcmd.respond_info("=== EGM Bridge Status ===")
        gcmd.respond_info(f"  Active: {self.is_active}")
        gcmd.respond_info(f"  Move Count: {self.move_count}")
        gcmd.respond_info(f"  Log Moves: {self.log_moves}")
        gcmd.respond_info(f"  Mode: {'Hardware' if self.toolhead else 'Simulator'}")
        gcmd.respond_info(f"  Position: {self.virtual_pos}")
        gcmd.respond_info("=========================")
    
    def cmd_EGM_ACTIVATE(self, gcmd):
        self.is_active = True
        gcmd.respond_info("EGM Bridge AKTIVIERT")
        self._log("Aktiviert")
    
    def cmd_EGM_DEACTIVATE(self, gcmd):
        self.is_active = False
        gcmd.respond_info("EGM Bridge DEAKTIVIERT")
        self._log("Deaktiviert")
    
    def cmd_EGM_TEST(self, gcmd):
        param = gcmd.get('PARAM', 'default')
        value = gcmd.get_float('VALUE', 0.0)
        gcmd.respond_info(f"EGM_TEST: PARAM={param}, VALUE={value}")
        self._log(f"Test: PARAM={param}, VALUE={value}")
    
    def cmd_EGM_MOVE(self, gcmd):
        x = gcmd.get_float('X', self.virtual_pos[0])
        y = gcmd.get_float('Y', self.virtual_pos[1])
        z = gcmd.get_float('Z', self.virtual_pos[2])
        e = gcmd.get_float('E', self.virtual_pos[3])
        f = gcmd.get_float('F', 3000.0)
        
        old = self.virtual_pos.copy()
        self.virtual_pos = [x, y, z, e]
        self.move_count += 1
        
        if self.log_moves:
            gcmd.respond_info(f"Move #{self.move_count}: {old} -> {self.virtual_pos}")
        self._log(f"Move: X={x:.2f} Y={y:.2f} Z={z:.2f} E={e:.3f} F={f}")
    
    def cmd_EGM_MOVE_LOG(self, gcmd):
        self.log_moves = not self.log_moves
        gcmd.respond_info(f"Move-Logging: {'AN' if self.log_moves else 'AUS'}")
    
    def get_status(self, eventtime):
        return {
            'is_active': self.is_active,
            'move_count': self.move_count,
            'position': self.virtual_pos,
        }

def load_config(config):
    return EgmBridge(config)
