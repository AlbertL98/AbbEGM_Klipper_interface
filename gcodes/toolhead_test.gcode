; =============================================================================
; Toolhead + Extruder Test G-Code
; Für Klipper Simulation — erzeugt Trapezsegmente über Toolhead-Trapq
; =============================================================================

; Position setzen ohne Homing (keine Endstops in Simulation)
SET_KINEMATIC_POSITION X=0 Y=0 Z=100

; Extruder vorbereiten
M83                    ; Relative Extrusion
M302 S0               ; Cold Extrusion erlauben

; Erste bewegung um Bridge startup nicht zu überfordern
G1 X0 Y0 Z10 F10

; --- Einfaches Quadrat mit Extrusion (simuliert eine Print-Layer) ---
; Geschwindigkeit F1800 = 30mm/s

G1 X150 Y50  Z10 E2.0 F1800   ; Linie 1: rechts
G1 X150 Y150 Z10 E2.0 F1800   ; Linie 2: hoch
G1 X50  Y150 Z10 E2.0 F1800   ; Linie 3: links
G1 X50  Y50  Z10 E2.0 F1800   ; Linie 4: zurück zum Start

; --- Zweite Layer, etwas schneller ---
G1 Z10.3 F600                  ; Z-Hop

G1 X150 Y50  Z10.3 E2.0 F2400 ; Linie 1: rechts (40mm/s)
G1 X150 Y150 Z10.3 E2.0 F2400 ; Linie 2: hoch
G1 X50  Y150 Z10.3 E2.0 F2400 ; Linie 3: links
G1 X50  Y50  Z10.3 E2.0 F2400 ; Linie 4: zurück

; --- Diagonale Moves (testet axes_r Richtungsvektoren) ---
G1 Z10.6 F600                  ; Z-Hop

G1 X200 Y200 Z10.6 E3.0 F3000 ; Diagonale 1
G1 X50  Y200 Z10.6 E2.0 F3000 ; Zurück links
G1 X200 Y50  Z10.6 E3.0 F3000 ; Diagonale 2
G1 X50  Y50  Z10.6 E2.0 F3000 ; Zurück zum Start

; --- Verschiedene Geschwindigkeiten (testet Trapez-Profile) ---
G1 Z10.9 F600                  ; Z-Hop

G1 X250 Y50  Z10.9 E1.5 F600  ; Langsam (10mm/s)
G1 X250 Y250 Z10.9 E3.0 F1800 ; Mittel (30mm/s)
G1 X50  Y250 Z10.9 E3.0 F6000 ; Schnell (100mm/s)
G1 X50  Y50  Z10.9 E3.0 F9000 ; Sehr schnell (150mm/s)

; Fertig
M400                           ; Warten bis Queue leer
