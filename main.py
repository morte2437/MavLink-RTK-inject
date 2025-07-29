#!/usr/bin/env -S python3 -u

import socket
import sys
import datetime
import base64
import time
import threading
import math
from optparse import OptionParser
from datetime import datetime, timezone, timedelta
import serial
from pynmeagps import NMEAReader
from pymavlink import mavutil

class MavlinkRTCMClient:
    def __init__(self,
                 gps_port='/dev/ttyUSB0',
                 gps_baud=460800,
                 mav_port='/dev/ttyAMA0',
                 mav_baud=921600,
                 gps_file_path=None,
                 verbose=True):

        self.verbose = verbose
        self.stream = serial.Serial(gps_port, gps_baud, timeout=3)
        self.nmr = NMEAReader(self.stream)
        self.mav = mavutil.mavlink_connection(mav_port, baud=mav_baud)
        if self.mav.wait_heartbeat():
            self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            5,  # rate (Hz)
            1   # start
        )
            print(self.mav.wait_heartbeat())
            print("MAVLink connected")

        self.gps_file_path = gps_file_path
        if self.gps_file_path:
            self.gps_file = open(self.gps_file_path, 'ab')
        else:
            self.gps_file = None

        self._stop = False
        self.mav_thread = threading.Thread(target=self._mavloop, daemon=True)
        self.mav_thread.start()

    def _mavloop(self):
        while not self._stop:
            #print("mavlooping")
            #print(self.mav.recv_match(type='GPS_STATUS' ,blcoking=True, timeout=1))
            msg = self.mav.recv_match(type='GPS_RTCM_DATA', timeout=2)
            if msg:
                #print("RTCM message Received")
                rtcm_bytes = msg.data[:msg.len]
                self.stream.write(rtcm_bytes)
                #print(self.stream.write(rtcm_bytes))
                if self.verbose:
                    print(f"[RTCM] Injected {msg.len} bytes")
                    
                    
                    
    def get_gps_week_and_ms(self, utc_dt):
        """
        Return (gps_week, ms_into_week) for a given UTC datetime.
        GPS epoch is 1980-01-06 00:00:00 UTC.
        """
        gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
        delta = utc_dt - gps_epoch
        total_seconds = delta.total_seconds()
        week = int(total_seconds // (7 * 24 * 3600))
        sow = total_seconds - week * 7 * 24 * 3600
        ms_of_week = int(sow * 1000)
        return week, ms_of_week
        
        
        
    def parse_gngga_sentence2(self, nmea_sentence):
        #print(nmea_sentence)
        parts = nmea_sentence.split(',')
        #print(parts)
        if len(parts) < 10:
            return None

        try:
            # --- 1) Parse UTC time field (hhmmss.sss) ---
            time_str = parts[1]                   # e.g. "005045.000"
            hh = int(time_str[0:2])               # hours
            mm = int(time_str[2:4])               # minutes
            ss = int(time_str[4:6])               # seconds
            frac = float("0." + time_str.split('.')[1]) if '.' in time_str else 0.0
            microsec = int(frac * 1e6)            # fractional ? microseconds

            # Build a full UTC datetime for today at that time
            today = datetime.now(timezone.utc).date()
            utc_dt = datetime(
                today.year, today.month, today.day,
                hh, mm, ss, microsec,
                tzinfo=timezone.utc
            )

            # --- 2) Compute timestamps ---
            time_usec = int(utc_dt.timestamp() * 1e6)     #us since Unix epoch
            gps_week, gps_ms = self.get_gps_week_and_ms(utc_dt)  # GPS-week & ms-of-week

            # --- 3) Parse Lat/Lon/Alt/Fix ---
            lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
            if parts[3] == 'S':
                lat = -lat

            lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
            if parts[5] == 'W':
                lon = -lon

            alt = float(parts[9]) if parts[9] else 0.0
            fix_quality = int(parts[6]) if parts[6].isdigit() else 0
            sats = int(parts[7]) if parts[7].isdigit() else 0
            hdop = float(parts[8])

        except (ValueError, IndexError):
            return None

        # Return everything you need for gps_input_send()
        return {
            'time_usec':    time_usec,
            'gps_week':     gps_week,
            'ms_of_week':   gps_ms,
            'lat':          lat,
            'lon':          lon,
            'alt':          alt,
            'fix_quality':  fix_quality,
            'satellites_visible': sats,
            'hdop' : hdop
        }
        
        
    def parse_gngga_sentence(self, nmea_sentence):
        parts = nmea_sentence.split(',')
        #print(nmea_sentence)
        if len(parts) < 10:
            return None
        try:

            lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
            if parts[3] == 'S':
                lat = -lat
            lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
            if parts[5] == 'W':
                lon = -lon
            alt = float(parts[9]) if parts[9] else 0.0
            fix_quality = int(parts[6]) if parts[6].isdigit() else 0
        except ValueError:
            return None
        return lat, lon, alt, fix_quality


    def parse_gnrmc_sentence(self, raw_data):
        """
        Parse a single $GNRMC NMEA sentence and return structured data.

        Returns None if the sentence is invalid or cannot be parsed.
        Adds North/East/Down velocity components (vn, ve, vd).
        """
        # Basic sanity check
            
        if not raw_data.startswith('$GNRMC'):
            print('no gnrmc detected')
            return None

        parts = nmea_sentence.split(',')
        # Must have at least time, status, lat, NS, lon, EW, speed, track, date
        if len(parts) < 10:
            return None

        try:
            # --- 1) Parse UTC time hhmmss.sss and date ddmmyy ---
            time_str = parts[1]
            hh = int(time_str[0:2])
            mm = int(time_str[2:4])
            ss = int(time_str[4:6])
            frac = float('0.' + time_str.split('.')[1]) if '.' in time_str else 0.0
            microsec = int(frac * 1e6)

            date_str = parts[9]
            day   = int(date_str[0:2])
            month = int(date_str[2:4])
            year  = int(date_str[4:6]) + 2000

            # Build UTC datetime from RMC fields
            utc_dt = datetime(year, month, day, hh, mm, ss, microsec, tzinfo=timezone.utc)

            # Convert to timestamps
            time_usec  = int(utc_dt.timestamp() * 1e6)
            gps_week, gps_ms = self.get_gps_week_and_ms(utc_dt)

            # --- 2) Parse status ---
            status = parts[2]

            # --- 3) Parse latitude/longitude ---
            raw_lat, lat_dir = parts[3], parts[4]
            lat_deg = float(raw_lat[:2]) + float(raw_lat[2:]) / 60.0
            if lat_dir == 'S': lat_deg = -lat_deg

            raw_lon, lon_dir = parts[5], parts[6]
            lon_deg = float(raw_lon[:3]) + float(raw_lon[3:]) / 60.0
            if lon_dir == 'W': lon_deg = -lon_deg

            # --- 4) Parse speed (knots) and track angle (degrees) ---
            speed_knots = float(parts[7]) if parts[7] else 0.0
            track_angle = float(parts[8]) if parts[8] else 0.0

            # --- 5) Compute NED velocity components ---
            speed_mps = speed_knots * 0.514444
            vn = speed_mps * math.cos(math.radians(track_angle))
            ve = speed_mps * math.sin(math.radians(track_angle))
            # Vertical velocity not provided by RMC; set None or compute via GGA differencing elsewhere
            vd = None

            # --- 6) Parse mode (optional, before checksum) ---
            mode = ''
            if len(parts) >= 12:
                fld = parts[12]
                mode = fld.split('*')[0] if '*' in fld else fld

        except (ValueError, IndexError):
            return None

        return {
            'time_usec':    time_usec,
            'gps_week':     gps_week,
            'ms_of_week':   gps_ms,
            'status':       status,
            'lat':          lat_deg,
            'lon':          lon_deg,
            'speed_knots':  speed_knots,
            'track_angle':  track_angle,
            'vn':           vn,
            've':           ve,
            'vd':           vd,
            'date':         date_str,
            'mode':         mode,
        }



    def readData(self):
        try:
            while True:
                (raw_data, parsed_data) = self.nmr.read()
                #print(raw_data)
                if b'GNGGA' in raw_data:
                    #print(raw_data.decode('ascii'))
                    nmea = raw_data.decode('ascii')
                    #print(nmea)
                    location = self.parse_gngga_sentence(nmea)
                    if location:
                        lat, lon, alt, fix_quality = location
                        fix_status = {
                            0: "Invalid",
                            1: "GPS fix",
                            2: "DGPS fix",
                            3: "PPS fix",
                            4: "RTK fixed",
                            5: "RTK float",
                            6: "Estimated",
                            7: "Manual input",
                            8: "Simulation"
                        }.get(fix_quality, "Unknown")
                        print(f"Parsed Location: {lat:.7f}, {lon:.7f}, Alt: {alt:.2f}m, Fix Status: {fix_status}")
                        #send gps data over mavlink
                        # gps_input_send: reports raw GPS measurements
                        data = self.parse_gngga_sentence2(nmea)
                        #if b'GNRMC' in self.nmr.read():
                            #print('rmc found')
                            #gnrmc = raw_data.decode('ascii')
                            #rmc_data = self.parse_gnrmc_sentence(gnrmc)
                            #print(rmc_data)
                        nmea_q = data['fix_quality']
                        if nmea_q == 4:
                            mav_fix = 6
                        elif nmea_q == 5:
                            mav_fix = 5   # == 5
                        elif nmea_q == 2:
                            mav_fix = 4        # == 4
                        elif nmea_q == 1:
                            mav_fix = 3      # == 3
                        else:
                            mav_fix = 1      # == 1
                        self.mav.mav.gps_input_send(
                        data['time_usec'], #time in microseconds us time_usec
                        2, #gps_id
                        #254, #ignore_flags I think 0 for dont ignore any
                        252,
                        data['ms_of_week'], #time_week_ms gps time from start of gps week(gpt says how many seconds since start of current gps week)
                        data['gps_week'], #time_week Gps week number gpt says how many whole wees have elapsed since jan 6 1980
                        mav_fix, #fix_type 0-1 no fix, 2 2d fix, 3 3d fix, 4 3d fix with dgps, 5 3d with rtk
                        int(data['lat']*1e7), #lat in degE7 (WGS84)
                        int(data['lon']*1e7), #lon in degE7 (WGS84)
                        data['alt'], #altitude MSL in meters m
                        data['hdop'], #hdop if unknown set to UINT16_MAX
                        65535, #vdop same as above
                        0, #vn velocity in north direction earth-fixed NED frame in m/s
                        0, #ve velocity in east direction earth fixed ned frame in m/s
                        0, #vd velocity in down direction earth-fixed ned frame in m/s
                        0, #speed_accuracy in m/s
                        0, #horiz_accuracy in meters
                        0, #vert_accuracy in meters
                        data['satellites_visible']  #satellites visible
                        )


                    if self.gps_file:
                        self.gps_file.write(raw_data)
                        self.gps_file.flush()
        except KeyboardInterrupt:
            print("Exiting...")
            self._stop = True
            if self.gps_file:
                self.gps_file.close()
            self.stream.close()

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("-g", "--gps", dest="gps_port", default="/dev/ttyUSB0", help="GPS output serial port")
    parser.add_option("-m", "--mav", dest="mav_port", default="/dev/ttyAMA0", help="MAVLink input serial port")
    parser.add_option("--gpsbaud", dest="gps_baud", type="int", default=460800, help="Baud rate for GPS port")
    parser.add_option("--mavbaud", dest="mav_baud", type="int", default=921600, help="Baud rate for MAVLink port")
    parser.add_option("-f", "--outputFile", dest="outputFile", default=None, help="Optional file to log GPS data")
    (options, args) = parser.parse_args()

    client = MavlinkRTCMClient(gps_port=options.gps_port,
                                gps_baud=options.gps_baud,
                                mav_port=options.mav_port,
                                mav_baud=options.mav_baud,
                                gps_file_path=options.outputFile)
    client.readData()
