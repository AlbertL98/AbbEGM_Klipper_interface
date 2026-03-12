import csv
import math
import sys


def sync_printer_data():
    rx_path = 'rx.csv'
    tx_path = 'tx.csv'
    out_path = 'data.csv'

    # Konstante "k" für dein Ratio-Score. Passe sie bei Bedarf an.
    K = 0.023

    print("Starte Synchronisation... Lese Streams von RX und TX.\n")

    try:
        with open(rx_path, 'r') as frx, open(tx_path, 'r') as ftx, open(out_path, 'w', newline='') as fout:
            # DictReader ist robust gegenüber vertauschten Spalten
            rx_reader = csv.DictReader(frx)
            tx_reader = csv.DictReader(ftx)

            # Spalten der neuen Datei definieren
            fieldnames = [
                'Timestamp', 'Soll_X', 'Soll_Y', 'Soll_Z',
                'Ist_X', 'Ist_Y', 'Ist_Z', 'Extruder_Pos',
                'Soll_Velocity_TCP', 'Ist_Velocity_TCP',
                'Ist_Velocity_Extruder', 'Error_TCP', 'Error_TCP_Speed', 'Ratio_Score'
            ]
            writer = csv.DictWriter(fout, fieldnames=fieldnames)
            writer.writeheader()

            # --- INITIALISIERUNG TX POINTER ---
            try:
                current_tx = next(tx_reader)
                next_tx = next(tx_reader)
            except StopIteration:
                print("FEHLER: TX Datei ist zu kurz oder leer.")
                return

            # --- STATE VARIABLEN FÜR RX GESCHWINDIGKEIT ---
            prev_rx_t = None
            prev_rx_x, prev_rx_y, prev_rx_z = None, None, None
            prev_rx_e = None
            prev_robo_t = None
            prev_ext_t = None
            prev_e = None

            row_counter = 0

            # Haupt-Schleife geht stur durch die Ist-Daten (RX)
            for rx_row in rx_reader:
                rx_t = float(rx_row['timestamp'])

                # 1. FIND THE NEAREST (TX an RX angleichen)
                # Wir rücken den TX-Zeiger vor, solange der nächste TX-Zeitpunkt
                # näher an unserer aktuellen RX-Zeit liegt.
                while next_tx is not None:
                    curr_tx_t = float(current_tx['timestamp'])
                    next_tx_t = float(next_tx['timestamp'])

                    if abs(next_tx_t - rx_t) <= abs(curr_tx_t - rx_t):
                        current_tx = next_tx  # Übernehme den besseren/nächsten Wert
                        try:
                            next_tx = next(tx_reader)
                        except StopIteration:
                            next_tx = None  # Ende der TX Datei erreicht
                    else:
                        break  # current_tx ist der nächstgelegene Punkt

                # 2. WERTE AUSLESEN
                rx_x, rx_y, rx_z = float(rx_row['x']), float(rx_row['y']), float(rx_row['z'])

                tx_x, tx_y, tx_z = float(current_tx['x']), float(current_tx['y']), float(current_tx['z'])
                tx_v_soll = float(current_tx['velocity'])

                robo_t = float(rx_row['robot_time'])
                if rx_row['e_value'] is not None:
                    exclude_extruder = False
                    rx_e = float(rx_row['e_value'])
                    ext_t = float(rx_row['extruder_age_ms'])
                else:
                    exclude_extruder = True
                    rx_e = 0
                    ext_t = 0

                # 3. GESCHWINDIGKEITEN BERECHNEN TCP
                v_tcp_ist = 0.0

                if prev_robo_t is not None:
                    dt = (robo_t - prev_robo_t) / 1000
                    if dt > 0:
                        dx, dy, dz = (rx_x - prev_rx_x), (rx_y - prev_rx_y), (rx_z - prev_rx_z)
                        v_tcp_ist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2) / dt

                prev_robo_t = robo_t

                # 3.5 GESCHWINDIGKEITEN BERECHNEN Extruder

                v_ext_ist = 0.0

                if prev_ext_t is not None:

                    if ext_t < prev_ext_t:
                        dt = ext_t + prev_ext_t
                        de = (rx_e - prev_rx_e) * 1000
                        v_ext_ist = de / dt
                    else:
                        v_ext_ist = prev_ext_v

                prev_ext_t = ext_t

                # 4. TCP ERROR BERECHNEN (Euklidischer 3D Abstand)
                err_tcp = math.sqrt((rx_x - tx_x) ** 2 + (rx_y - tx_y) ** 2 + (rx_z - tx_z) ** 2)

                err_tcp_v = v_tcp_ist - tx_v_soll

                # 5. RATIO SCORE BERECHNEN log(v_tcp * k / v_ext)
                # Sicherheit gegen Division durch 0 oder Logarithmus von 0 / negativen Zahlen (z.B. bei Retracts)
                ratio_score = 0.0
                if not exclude_extruder:
                    try:
                        ratio_score = (v_ext_ist-(v_tcp_ist * K)) / (abs(v_ext_ist) + abs(v_tcp_ist * K) + 0.0000001)
                    except ValueError:
                        pass

                # 6. IN CSV SCHREIBEN
                writer.writerow({
                    'Timestamp': f"{rx_t:.6f}",
                    'Soll_X': f"{tx_x:.4f}", 'Soll_Y': f"{tx_y:.4f}", 'Soll_Z': f"{tx_z:.4f}",
                    'Ist_X': f"{rx_x:.4f}", 'Ist_Y': f"{rx_y:.4f}", 'Ist_Z': f"{rx_z:.4f}",
                    'Extruder_Pos': f"{rx_e:.6f}",
                    'Soll_Velocity_TCP': f"{tx_v_soll:.3f}",
                    'Ist_Velocity_TCP': f"{v_tcp_ist:.3f}",
                    'Ist_Velocity_Extruder': f"{v_ext_ist:.3f}",
                    'Error_TCP': f"{err_tcp:.5f}",
                    'Error_TCP_Speed': f"{err_tcp_v:.5f}",
                    'Ratio_Score': f"{ratio_score:.4f}"
                })

                # 7. TERMINAL AUSGABE
                # Update nur jede 250. Zeile, um den I/O Flaschenhals im Terminal zu verhindern
                #if row_counter % 1 == 0:

                sys.stdout.write(
                    f"\n[{row_counter:07d}] t:{rx_t:.2f} | TX:{tx_x:.1f},{tx_y:.1f} -> RX:{rx_x:.1f},{rx_y:.1f} | Err:{err_tcp:.3f} | vT:{v_tcp_ist:.1f} | Sc:{ratio_score:.2f}    ")
                sys.stdout.flush()

                # Status für den nächsten Durchlauf updaten
                prev_rx_t = rx_t
                prev_rx_x, prev_rx_y, prev_rx_z = rx_x, rx_y, rx_z
                prev_rx_e = rx_e
                prev_ext_v = v_ext_ist
                row_counter += 1

        print(f"\n\nFertig! {row_counter} Zeilen verarbeitet. Ergebnis wurde in '{out_path}' gespeichert.")

    except FileNotFoundError as e:
        print(f"\nFEHLER: Datei nicht gefunden. Stelle sicher, dass rx.csv und tx.csv existieren. ({e})")
    except Exception as e:
        print(f"\nEIN UNERWARTETER FEHLER IST AUFGETRETEN: {e}")


if __name__ == "__main__":
    sync_printer_data()