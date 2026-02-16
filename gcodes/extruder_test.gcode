; =============================================================================
; Extruder Queue Validation Test
; Für Klipper Simulation mit Host-MCU (keine echte Hardware)
; =============================================================================

; Position setzen ohne Homing (wir haben keine echten Endstops)
SET_KINEMATIC_POSITION X=250 Y=250 Z=150

; Extruder-Temperatur: min_extrude_temp umgehen
; temperature_host liest die CPU-Temp (~40-60°C im Container)
; Wir setzen min_extrude_temp auf 0 damit wir kalt extrudieren können
SET_HEATER_TEMPERATURE HEATER=extruder TARGET=0
M302 S0  ; Cold extrusion erlauben (min_extrude_temp = 0)

; --- Extruder Test Moves (Original-GCode) ---
M83           ; Relative Extrusion
G1 E1.0 F60  ; 1mm bei 1mm/s
G1 E2.0 F120 ; 2mm bei 2mm/s
G1 E-0.8 F180 ; Retraction 0.8mm bei 3mm/s
G1 E5.0 F600 ; 5mm bei 10mm/s

; Kurze Pause damit alles abgearbeitet wird
G4 P500       ; 500ms warten

; --- Ende ---
M400          ; Warten bis Queue leer
