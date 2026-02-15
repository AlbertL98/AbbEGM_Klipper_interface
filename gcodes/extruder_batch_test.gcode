; =============================================================================
; Extruder Queue Validation — Batch Mode Test
; Für: klippy.py -i THIS_FILE -o test.serial -d out/klipper.dict
; =============================================================================

; Position setzen ohne Homing (Batch-Modus hat keine Endstops)
SET_KINEMATIC_POSITION X=250 Y=250 Z=150

; Relative Extrusion
M83

; --- Test Moves: verschiedene Geschwindigkeiten ---
G1 E1.0 F60    ; 1mm bei 1mm/s   (langsam)
G1 E2.0 F120   ; 2mm bei 2mm/s   (mittel)
G1 E-0.8 F180  ; Retraction 0.8mm bei 3mm/s
G1 E5.0 F600   ; 5mm bei 10mm/s  (schnell)

; --- XY + E kombiniert (realistischer Print-Move) ---
G1 X260 Y250 E0.5 F1200   ; 10mm X-Move mit Extrusion
G1 X260 Y260 E0.5 F1200   ; 10mm Y-Move mit Extrusion
G1 X250 Y260 E0.5 F1200   ; zurück
G1 X250 Y250 E0.5 F1200   ; Quadrat schließen

; Warten bis Queue leer
G4 P500
M400
