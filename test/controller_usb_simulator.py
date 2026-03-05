import argparse
import random
import time
import serial
import threading
from datetime import datetime

# Variable globale pour l'arrêt propre des threads
running = True

def sender_thread(ser, interval, verbose, logfile):
    """Thread dédié à l'envoi des données d'orientation à fréquence fixe."""
    global running
    iteration = 0
    current_pitch, current_roll, current_yaw = 0.0, 0.0, 0.0
    
    print(f"[*] Sender thread started (Interval: {interval}s)")
    
    while running:
        iteration += 1
        # Simulation de mouvement fluide
        current_pitch = max(-30.0, min(30.0, current_pitch + random.uniform(-0.5, 0.5)))
        current_roll = max(-30.0, min(30.0, current_roll + random.uniform(-0.5, 0.5)))
        current_yaw = (current_yaw + random.uniform(-1.0, 1.0) + 360.0) % 360.0
        
        line = f"O: {current_pitch:.2f} {current_roll:.2f} {current_yaw:.2f}\n"
        
        try:
            ser.write(line.encode('ascii'))
            if verbose:
                print(f"[{iteration}] TX: {line.strip()}")
            if logfile:
                logfile.write(f"TX: {line.strip()}\n")
        except Exception as e:
            print(f"Error sending: {e}")
            break
            
        time.sleep(interval)

def receiver_thread(ser, verbose, logfile):
    """Thread dédié à la réception et au parsing des commandes moteurs."""
    global running
    print("[*] Receiver thread started")
    
    while running:
        if ser.in_waiting > 0:
            try:
                # Lecture d'une ligne complète
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                
                if verbose:
                    print(f"  Recv Raw: {line}")
                
                # Parsing du format M<index>:<speed>
                if line.startswith('M') and ':' in line:
                    try:
                        parts = line[1:].split(':')
                        idx = int(parts[0])
                        speed = float(parts[1])
                        msg = f"  ✓ Motor {idx}: {speed:.1f}"
                        print(msg)
                        if logfile: logfile.write(msg + "\n")
                    except:
                        print(f"  ✗ Parsing error: {line}")
                elif not line.startswith('O:'): # Ignore les échos si présents
                    print(f"  ? Msg: {line}")
                    
            except Exception as e:
                print(f"Error receiving: {e}")
                break
        else:
            time.sleep(0.001) # Petite pause pour ne pas saturer le CPU

def main():
    parser = argparse.ArgumentParser(description="Multi-threaded USB simulator")
    parser.add_argument("--port", required=True, help="serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--interval", type=float, default=0.01)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        ser.flush() # Vide les buffers au démarrage
    except Exception as e:
        print(f"Could not open port: {e}")
        return

    logfile = None # Tu peux rajouter la logique de log ici si besoin

    # Création des threads
    t_send = threading.Thread(target=sender_thread, args=(ser, args.interval, args.verbose, logfile))
    t_recv = threading.Thread(target=receiver_thread, args=(ser, args.verbose, logfile))

    # Lancement
    t_send.start()
    t_recv.start()

    try:
        while True:
            time.sleep(0.1) # Le thread principal attend le Ctrl+C
    except KeyboardInterrupt:
        print("\nStopping...")
        global running
        running = False
        t_send.join()
        t_recv.join()
        ser.close()
        print("Done.")

if __name__ == '__main__':
    main()